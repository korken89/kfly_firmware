/* *
 *
 * Abstraction Layer for RC Outputs
 *
 * */

#include "ch.h"
#include "hal.h"
#include "eicu.h"
#include "flash_save.h"
#include "rc_input.h"
#include "trigonometry.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/
static void RawInputToCalibratedInput(void);
static void cppm_callback(EICUDriver *eicup, eicuchannel_t channel);
static void rssi_callback(EICUDriver *eicup, eicuchannel_t channel);
static void pwm_callback(EICUDriver *eicup, eicuchannel_t channel);
static void vt_no_connection_timeout_callback(void *p);

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Data holder for the RC input.
 */
rcinput_data_t rcinput_data;

/**
 * @brief   Role lookup table.
 */
rcinput_role_lookup_t rcinput_role_lookup;

/**
 * @brief   Data holder for the RC input settings.
 */
rcinput_settings_t rcinput_settings;

/**
 * @brief   Event source for the RC input, indicates new measurements and
 *          connection status.
 */
EVENTSOURCE_DECL(rcinput_es);

/**
 * @brief   Timer for checking time out of the RC input.
 */
virtual_timer_t rcinput_timeout_vt;

/* EICU Configuration for CPPM, RSSI and PWM inputs */
static const EICU_IC_Settings cppmsettings = {
    EICU_INPUT_ACTIVE_LOW,      /* Edge detection setting */
    cppm_callback               /* Detection callback */
};
static const EICUConfig cppm_rcinputcfg = {
    EICU_INPUT_EDGE,            /* Input type configuration */
    RCINPUT_CAPTURE_TIMER_RATE, /* Timer clock in Hz */
    {
        &cppmsettings,          /* Input capture settings pointer 1 */
        NULL,                   /* Input capture settings pointer 2 */
        NULL,                   /* Input capture settings pointer 3 */
        NULL                    /* Input capture settings pointer 4 */
    },
    NULL,                       /* Period capture callback */
    NULL,                       /* Overflow capture callback */
    0                           /* DEIR init data */
};

static const EICU_IC_Settings rssisettings = {
    EICU_INPUT_ACTIVE_HIGH,     /* Edge detection setting */
    NULL                        /* Detection callback */
};
static const EICUConfig rssi_rcinputcfg = {
    EICU_INPUT_PWM,             /* Input type configuration */
    RCINPUT_CAPTURE_TIMER_RATE, /* Timer clock in Hz */
    {
        NULL,                   /* Input capture settings pointer 1 */
        &rssisettings,          /* Input capture settings pointer 2 */
        NULL,                   /* Input capture settings pointer 3 */
        NULL                    /* Input capture settings pointer 4 */
    },
    rssi_callback,              /* Period capture callback */
    NULL,                       /* Overflow capture callback */
    0                           /* DEIR init data */
};

static const EICU_IC_Settings pwmsettings = {
    EICU_INPUT_ACTIVE_HIGH,     /* Edge detection setting */
    pwm_callback                /* Detection callback */
};
static const EICUConfig pwm_rcinputcfg_1 = {
    EICU_INPUT_PULSE,           /* Input type configuration */
    RCINPUT_CAPTURE_TIMER_RATE, /* Timer clock in Hz */
    {
        &pwmsettings,           /* Input capture settings pointer 1 */
        &pwmsettings,           /* Input capture settings pointer 2 */
        NULL,                   /* Input capture settings pointer 3 */
        NULL                    /* Input capture settings pointer 4 */
    },
    NULL,                       /* Period capture callback */
    NULL,                       /* Overflow capture callback */
    0                           /* DEIR init data */
};

static const EICUConfig pwm_rcinputcfg_2 = {
    EICU_INPUT_PULSE,           /* Input type configuration */
    RCINPUT_CAPTURE_TIMER_RATE, /* Timer clock in Hz */
    {
        NULL,                   /* Input capture settings pointer 1 */
        NULL,                   /* Input capture settings pointer 2 */
        &pwmsettings,           /* Input capture settings pointer 3 */
        &pwmsettings            /* Input capture settings pointer 4 */
    },
    NULL,                       /* Period capture callback */
    NULL,                       /* Overflow capture callback */
    0                           /* DEIR init data */
};

uint16_t rssi_counter = 0;
THD_WORKING_AREA(waThreadRCInputFlashSave, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief           Thread for the flash save operation.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadRCInputFlashSave, arg)
{
    (void)arg;

    /* Event registration for new estimation */
    event_listener_t el;

    /* Set thread name */
    chRegSetThreadName("RCInput FlashSave");

    /* Register to new estimation */
    chEvtRegisterMask(ptrGetFlashSaveEventSource(),
                      &el,
                      FLASHSAVE_SAVE_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation */
        chEvtWaitOne(FLASHSAVE_SAVE_EVENTMASK);

        /* Save RC input settings to flash */
        FlashSave_Write(FlashSave_STR2ID("RCIN"),
                        true,
                        (uint8_t *)&rcinput_settings,
                        RCINPUT_SETTINGS_SIZE);
    }
}

/**
 * @brief               Parses CPPM inputs. This function runs inside a
 *                      osalSysLockFromISR.
 *
 *
 * @param[in/out] data  Pointer to RC Input data structure.
 * @param[in] capture   The value of the latest input capture.
 */
static void ParseCPPMInput(const uint32_t capture)
{
    static uint16_t cppm_count = 0; /* Current CPPM channel */

    if (rcinput_data.active_connection.value == true)
    {
        /* If the capture is larger than the time for the SYNC, reset counter */
        if (capture > RCINPUT_CPPM_SYNC_LIMIT_MIN)
        {
            /* Save the current number of active channels */
            rcinput_data.number_active_connections = cppm_count;

            /* Parse new input to calibrated values. */
            RawInputToCalibratedInput();

            /* Reset CPPM counter */
            cppm_count = 0;

            /* Reset timeout and broadcast new input */
            chVTSetI(&rcinput_timeout_vt,
                     MS2ST(RCINPUT_NO_CON_TIMEOUT_MS),
                     vt_no_connection_timeout_callback,
                     NULL);
            chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_NEWINPUT_EVENTMASK);
        }
        else
        {
            /* If no sync has been detected */
            if (cppm_count >= RCINPUT_MAX_NUMBER_OF_INPUTS)
            {
                /* Reset connection */
                rcinput_data.active_connection.value = false;

                /* Disable timeout timer and broadcast connection lost */
                chVTResetI(&rcinput_timeout_vt);
                chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_LOST_EVENTMASK);
            }
            else
            {
                /* Write the capture value to the data structure */
                rcinput_data.value[cppm_count] = capture;

                /* Increase the current CPPM channel */
                cppm_count++;
            }
        }
    }
    else if ((capture > RCINPUT_CPPM_SYNC_LIMIT_MIN) && \
             (capture < RCINPUT_CPPM_SYNC_LIMIT_MAX) && \
             (rssi_counter < RCINPUT_RSSI_TIMEOUT))
    {
        /* Sync found, reset CPPM counter and activate connection */
        cppm_count = 0;
        rcinput_data.active_connection.value = true;

        /* Enable timeout timer and broadcast connection active */
        chVTSetI(&rcinput_timeout_vt,
                 MS2ST(RCINPUT_NO_CON_TIMEOUT_MS),
                 vt_no_connection_timeout_callback,
                 NULL);
        chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_ACTIVE_EVENTMASK);
    }
}

/**
 * @brief               Parses RSSI inputs. This function runs inside a
 *                      osalSysLockFromISR.
 *
 * @param[in/out] data  Pointer to RC Input data structure.
 * @param[in] width     The value of the latest width capture.
 * @param[in] period    The value of the latest period capture.
 */
static void ParseRSSIInput(const uint32_t width, const uint32_t period)
{
    /* If there is valid data, save it else reset RSSI values */
    if (period != 0)
    {
        /* Get the Input Capture value and calculate PWM frequency */
        rcinput_data.rssi_frequency = RCINPUT_CAPTURE_TIMER_RATE / period;

        /* Convert the RSSI PWM to percent */
        rcinput_data.rssi = (width * 100) / period;

        /* Check so the RSSI is above the threshold */
        if (rcinput_data.rssi < RCINPUT_RSSI_THRESHOLD_PERCENT)
        {
            /* If the RSSI is below the threshold count until timeout */
            if (rssi_counter > RCINPUT_RSSI_TIMEOUT)
            {
                rcinput_data.active_connection.value = false;

                /* Disable timeout timer and broadcast connection lost */
                chVTResetI(&rcinput_timeout_vt);
                chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_LOST_EVENTMASK);
            }
            else
                rssi_counter++;
        }
        else
            rssi_counter = 0;
    }
    else
    {
        rcinput_data.rssi = 0;
        rcinput_data.rssi_frequency = 0;
    }
}

/**
 * @brief               Reset a RCInput data structure.
 *
 * @param[in/out] data  Pointer to RC Input data structure.
 */
static void RCInputDataReset(void)
{
    int i;

    rcinput_data.active_connection.value = false;
    rcinput_data.number_active_connections = 0;
    rcinput_data.rssi = 0;
    rcinput_data.rssi_frequency = 0;

    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
    {
        rcinput_data.value[i] = 0;
        rcinput_data.calibrated_values.calibrated_value[i] = 0;
    }

    for (i = 0; i < RCINPUT_NUMBER_OF_SWITCHES; i++)
        rcinput_data.calibrated_values.switches[i] = RCINPUT_SWITCH_UNDEFINED;
}

/**
 * @brief               Reset a RCInput data structure.
 *
 * @param[in/out] data  Pointer to RC Input data structure.
 */
static void RCInputSettingsReset(void)
{
    int i;

    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
    {
        rcinput_settings.role[i]             = RCINPUT_ROLE_OFF;
        rcinput_settings.type[i]             = RCINPUT_TYPE_ANALOG;
        rcinput_settings.ch_reverse[i].value = false;
        rcinput_settings.ch_bottom[i]        = 1000;
        rcinput_settings.ch_center[i]        = 1500;
        rcinput_settings.ch_top[i]           = 2000;
    }

    rcinput_settings.mode = RCINPUT_MODE_CPPM_INPUT;
    rcinput_settings.use_rssi.value = false;
}

/**
 * @brief   Checks the settings so the roles are not duplicate and sets
 *          duplicates to zero.
 */
static void ValidateRoleSettings(void)
{

    int i, j, cnt;

    for (i = 0; i < RCINPUT_ROLE_MAX; i++)
    {
        cnt = 0;

        for (j = 0; j < RCINPUT_MAX_NUMBER_OF_INPUTS; j++)
            if (rcinput_settings.role[j] == (rcinput_role_selector_t)i)
                cnt++;

        if (cnt > 1)
            for (j = 0; j < RCINPUT_MAX_NUMBER_OF_INPUTS; j++)
                rcinput_settings.role[j] = RCINPUT_ROLE_OFF;
    }
}

/**
 * @brief   Generates the role -> index lookup table.
 */
static void GenerateRoleLookupTable(void)
{
    int i;
    rcinput_role_selector_t role;

    /* Validate the role settings before generating the lookup table. */
    ValidateRoleSettings();

    /* For each role associated with a channel, generate
       the inverse lookup table */
    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
    {
        role = rcinput_settings.role[i];

        if (role < RCINPUT_ROLE_MAX)
            rcinput_role_lookup.index[(uint32_t)role] = i;
        else
            rcinput_role_lookup.index[(uint32_t)role] = 0xff;
    }
}

/**
 * @brief               Converts role to corresponding array index.
 *
 * @param[in] role      Input role.
 * @return              Corresponding array index. Note: Returns 0xff for
 * @note                Returns 0xff for invalid roles.
 */
static uint8_t RoleToIndex(rcinput_role_selector_t role)
{
    /* Get the index of the associated role. */
    if (role < RCINPUT_ROLE_MAX)
        return rcinput_role_lookup.index[role];
    else
        return 0xff;
}

/**
 * @brief           Get the input level of the corresponding role and returns
 *                  it in the span of -1.0 to 1.0 or 0.0 to 1.0.
 *
 * @param[in] role  Input role for the RC input.
 * @return          The current input value of the corresponding role.
 */
static float GetAnalogLevel(uint32_t idx)
{
    int32_t value;
    float level;

    /* Get the raw value from the raw data structure */
    value = rcinput_data.value[idx];

    /* Check so the data and input is valid */
    if (value == 0)
        return 0.0f;

    /* Remove the center offset */
    level = (float)(value - rcinput_settings.ch_center[idx]);

    /* If it is larger than zero */
    if (level > 0.0f)
    {
        /* If the settings does not allow positive output */
        if (rcinput_settings.ch_center[idx] == rcinput_settings.ch_top[idx])
            level = 0.0f;
        else /* Use the calibration to calculate the position */
            level = level / (float)(rcinput_settings.ch_top[idx] -
                                    rcinput_settings.ch_center[idx]);
    }
    /* If it is smaller than zero */
    else if (level < 0.0f)
    {
        /* If the settings does not allow negative output */
        if (rcinput_settings.ch_center[idx] == rcinput_settings.ch_bottom[idx])
            level = 0.0f;
        else /* Use the calibration to calculate the position */
            level = level / (float)(rcinput_settings.ch_center[idx] -
                                    rcinput_settings.ch_bottom[idx]);
    }

    if (rcinput_settings.ch_reverse[idx].value == true)
    {
        if (rcinput_settings.ch_center[idx] == rcinput_settings.ch_bottom[idx])
            return bound(1.0f, 0.0f, 1.0f - level); /* Invert a positive only channel */
        if (rcinput_settings.ch_center[idx] == rcinput_settings.ch_top[idx])
            return bound(0.0f, -1.0f, -1.0f - level); /* Invert a negative only channel */
        else
            level = -level; /* Invert a normal channel */
    }

    return bound(1.0f, -1.0f, level);
}

/**
 * @brief           Get the input level of switches, if it is a switch.
 *
 * @param[in] role  Input role for the RC input.
 * @return          The current switch state of the corresponding role.
 */
static rcinput_switch_position_t GetSwitchState(rcinput_role_selector_t role)
{
    uint8_t idx;
    float value;
    rcinput_type_selector_t type;

    /* Check the validity of the index */
    if ((role < RCINPUT_ROLE_SWITCHES_START) ||
        (role >= RCINPUT_ROLE_MAX) ||
        (rcinput_data.active_connection.value == false))
        return RCINPUT_SWITCH_UNDEFINED;

    /* Get the position in the array for the requested role */
    idx = RoleToIndex(role);

    if (idx >= RCINPUT_MAX_NUMBER_OF_INPUTS)
        return RCINPUT_SWITCH_UNDEFINED;

    /* Get the raw value from the raw data structure */
    value = rcinput_data.calibrated_values.calibrated_value[idx];
    type = rcinput_settings.type[idx];

    /* Check so the data and input is valid */
    if (type == RCINPUT_TYPE_ANALOG)
        return RCINPUT_SWITCH_UNDEFINED;

    if (type == RCINPUT_TYPE_3_STATE)
    {
        /* 3 state switch. */

        /* Calculate the the center span as 1/5 of total span. */
        const float center = 2.0f / 5.0f;

        if (value > center)
        {
            /* Switch at bottom, but check for channel reversal. */
            if (rcinput_settings.ch_reverse[idx].value == true)
                return RCINPUT_SWITCH_POSITION_BOTTOM;
            else
                return RCINPUT_SWITCH_POSITION_TOP;
        }
        else if (value < -center)
        {
            /* Switch at top, but check for channel reversal. */
            if (rcinput_settings.ch_reverse[idx].value == true)
                return RCINPUT_SWITCH_POSITION_TOP;
            else
                return RCINPUT_SWITCH_POSITION_BOTTOM;
        }
        else
            return RCINPUT_SWITCH_POSITION_CENTER;
    }
    else
    {
        /* 2 state switch. */
        if (value < 0.0f)
        {
            /* Switch at bottom, but check for channel reversal. */
            if (rcinput_settings.ch_reverse[idx].value == true)
                return RCINPUT_SWITCH_POSITION_TOP;
            else
                return RCINPUT_SWITCH_POSITION_BOTTOM;
        }
        else
        {
            /* Switch at top, but check for channel reversal. */
            if (rcinput_settings.ch_reverse[idx].value == true)
                return RCINPUT_SWITCH_POSITION_BOTTOM;
            else
                return RCINPUT_SWITCH_POSITION_TOP;
        }
    }
}

/**
 * @brief               Parses raw inputs to calibrated inputs for later usage.
 *
 * @param[in/out] data  Pointer to RC Input data structure.
 */
static void RawInputToCalibratedInput(void)
{
    uint32_t i;

    /* Convert all analog values. */
    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
        rcinput_data.calibrated_values.calibrated_value[i] = GetAnalogLevel(i);

    /* Convert switch values. */
    for (i = RCINPUT_ROLE_SWITCHES_START; i < RCINPUT_ROLE_MAX; i++)
        rcinput_data.calibrated_values.switches[i -
            RCINPUT_ROLE_SWITCHES_START] = GetSwitchState(i);
}

/**
 * @brief           Timeout callback for RC Input connection.
 * @details         This callback in invoked when neither the CPPM nor PWM
 *                  input has been invoked for a certain amount of time. This
 *                  is to detect if the cables have come loose.
 *
 * @param[in] p     Input parameter (unused).
 */
static void vt_no_connection_timeout_callback(void *p)
{
    (void)p;

    osalSysLockFromISR();

    /* Reset all inputs. */
    RCInputDataReset();

    chVTResetI(&rcinput_timeout_vt);
    chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_LOST_EVENTMASK);

    osalSysUnlockFromISR();
}

/**
 * @brief               Callback for a new CPPM capture.
 *
 * @param[out] eicup    Pointer to the EICU driver.
 * @param[in] channel   Channel that detected the input capture.
 */
static void cppm_callback(EICUDriver *eicup, eicuchannel_t channel)
{
    uint32_t capture, last_count;

    osalSysLockFromISR();

    capture = eicuGetWidth(eicup, channel);
    last_count = eicup->last_count[0];
    eicup->last_count[0] = capture;

    if (capture > last_count)       /* No overflow */
        capture = capture - last_count;
    else if (capture < last_count)  /* Timer overflow */
        capture = ((0xFFFF - last_count) + capture);

    ParseCPPMInput(capture);

    osalSysUnlockFromISR();
}

/**
 * @brief               Callback for a new RSSI capture.
 *
 * @param[out] eicup    Pointer to the EICU driver.
 * @param[in] channel   Channel that detected the input capture.
 */
static void rssi_callback(EICUDriver *eicup, eicuchannel_t channel)
{
    (void)channel;

    osalSysLockFromISR();

    ParseRSSIInput(eicuGetWidth(eicup, EICU_CHANNEL_2),
                   eicuGetPeriod(eicup));

    osalSysUnlockFromISR();
}

/**
 * @brief               Callback for a new PWM capture.
 *
 * @param[out] eicup    Pointer to the EICU driver.
 * @param[in] channel   Channel that detected the input capture.
 */
static void pwm_callback(EICUDriver *eicup, eicuchannel_t channel)
{
    (void)channel;

    /* TODO: Implement this!!! */
    /* TODO: Parse new input to calibrated values. */

    osalSysLockFromISR();

    /* Checl from which PWM unit the pulse came from. */
    if (eicup == &EICUD3)
    {
    }
    else if (eicup == &EICUD9)
    {
    }
    else if (eicup == &EICUD12)
    {
    }

    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief           Initializes the RC input module.
 */
void RCInputInit(void)
{
    /* Initialize the RC Input event source */
    osalEventObjectInit(&rcinput_es);

    /* Reset data structures */
    RCInputSettingsReset();

    /* Read RC input settings from flash */
    FlashSave_Read(FlashSave_STR2ID("RCIN"),
                   (uint8_t *)&rcinput_settings,
                   RCINPUT_SETTINGS_SIZE);

    if (RCInputInitialization() != MSG_OK)
        osalSysHalt("RC input initialization failed.");

    /* Start the Flash Save thread */
    chThdCreateStatic(waThreadRCInputFlashSave,
                      sizeof(waThreadRCInputFlashSave),
                      NORMALPRIO,
                      ThreadRCInputFlashSave,
                      NULL);
}

/**
 * @brief           Initializes RC inputs.
 *
 * @return          MSG_OK if the initialization was successful.
 */
msg_t RCInputInitialization(void)
{
    msg_t status = MSG_OK;

    /* Reset data structures before init. */
    RCInputDataReset();

    /* If the EICU driver was already in use, disable it */
    if (EICUD3.state != EICU_STOP)
        eicuDisable(&EICUD3);
    if (EICUD9.state != EICU_STOP)
        eicuDisable(&EICUD9);
    if (EICUD12.state != EICU_STOP)
        eicuDisable(&EICUD12);

    /* Configure the input capture unit */
    if (rcinput_settings.mode == RCINPUT_MODE_CPPM_INPUT)
    {
        /* Start and enable the EICU driver */
        eicuStart(&EICUD9, &cppm_rcinputcfg);
        eicuEnable(&EICUD9);

        if (rcinput_settings.use_rssi.value == true)
        {
            eicuStart(&EICUD12, &rssi_rcinputcfg);
            eicuEnable(&EICUD12);
        }
    }
    else if (rcinput_settings.mode == RCINPUT_MODE_PWM_INPUT)
    {
        /* Start and enable the EICU driver */
        eicuStart(&EICUD3, &pwm_rcinputcfg_2);
        eicuStart(&EICUD9, &pwm_rcinputcfg_1);
        eicuStart(&EICUD12, &pwm_rcinputcfg_1);
        eicuEnable(&EICUD3);
        eicuEnable(&EICUD9);
        eicuEnable(&EICUD12);
    }
    else /* Invalid input, return error */
        return MSG_RESET;

    /* Gernerate the role lookup table */
    GenerateRoleLookupTable();

    return status;
}

/**
 * @brief           Gets the current analog level of the requested role.
 *
 * @param[in] role  The role to get the value of.
 * @return          The analog value.
 */
float RCInputGetInputLevel(const rcinput_role_selector_t role)
{
    uint8_t idx;

    /* Get the position in the array for the requested role */
    idx = RoleToIndex(role);

    /* Check the validity of the index */
    if ((idx >= RCINPUT_MAX_NUMBER_OF_INPUTS) ||
        (rcinput_data.active_connection.value == false))
        return 0.0f;
    else
        return rcinput_data.calibrated_values.calibrated_value[idx];
}

/**
 * @brief           Gets the current state of the requested switch.
 *
 * @param[in] role  The switch role to get the state of.
 * @return          The switch position.
 */
rcinput_switch_position_t RCInputGetSwitchState(const rcinput_role_selector_t role)
{
    /* Check the validity of the index */
    if ((role < RCINPUT_ROLE_SWITCHES_START) ||
        (role >= RCINPUT_ROLE_MAX))
        return RCINPUT_SWITCH_NOT_SWITCH;
    else if (rcinput_data.active_connection.value == false)
        return RCINPUT_SWITCH_UNDEFINED;
    else
        return rcinput_data.
               calibrated_values.
               switches[role - RCINPUT_ROLE_SWITCHES_START];
}

/**
 * @brief               Parses a payload from the serial communication for
 *                      all the RC input settings and calibration.
 *
 * @param[in] payload   Pointer to the payload location.
 * @param[in] size      Size of the payload.
 */
void vParseSetRCInputSettings(const uint8_t *payload,
                              const size_t data_length)
{
    if (data_length == RCINPUT_SETTINGS_SIZE)
    {
        osalSysLock();

        /* Save the data */
        memcpy((uint8_t *)ptrGetRCInputSettings(),
               payload,
               RCINPUT_SETTINGS_SIZE);

        osalSysUnlock();
    }

    /* Reinitialize the RC Input module */
    RCInputInitialization();
}

/**
 * @brief           Return the current connection status of the RC Input.
 *
 * @return          Return true if there is a connection, else false.
 */
bool bActiveRCInputConnection(void)
{
    return rcinput_data.active_connection.value;
}

/**
 * @brief           Return the pointer to the RC Input data.
 *
 * @return          Pointer to the RC input data.
 */
rcinput_data_t *ptrGetRCInputData(void)
{
    return &rcinput_data;
}

/**
 * @brief           Return the pointer to the RC Input settings.
 *
 * @return          Pointer to the RC input settings.
 */
rcinput_settings_t *ptrGetRCInputSettings(void)
{
    return &rcinput_settings;
}

/**
 * @brief           Return the pointer to the RC Input event source.
 *
 * @return          Pointer to the RC input event source.
 */
event_source_t *ptrGetRCInputEventSource(void)
{
    return &rcinput_es;
}
