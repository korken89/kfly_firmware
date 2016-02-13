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
rcinput_data_t rcinput_data;
uint64_t role_lookup;
rcinput_settings_t rcinput_settings;
EVENTSOURCE_DECL(rcinput_es);
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
 * @param[out] data     Pointer to RC Input data structure.
 * @param[in] capture   The value of the latest input capture.
 */
static void ParseCPPMInput(rcinput_data_t *data,
                           uint32_t capture)
{
    static uint16_t cppm_count = 0; /* Current CPPM channel */

    if (data->active_connection == TRUE)
    {
        /* If the capture is larger than the time for the SYNC, reset counter */
        if (capture > RCINPUT_CPPM_SYNC_LIMIT_MIN)
        {
            /* Save the current number of active channels */
            data->number_active_connections = cppm_count;

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
                data->active_connection = FALSE;

                /* Disable timeout timer and broadcast connection lost */
                chVTResetI(&rcinput_timeout_vt);
                chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_LOST_EVENTMASK);
            }
            else
            {
                /* Write the capture value to the data structure */
                data->value[cppm_count] = capture;

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
        data->active_connection = TRUE;

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
 * @param[out] data     Pointer to RC Input data structure.
 * @param[in] width     The value of the latest width capture.
 * @param[in] period    The value of the latest period capture.
 */
static void ParseRSSIInput(rcinput_data_t *data,
                           uint32_t width,
                           uint32_t period)
{
    /* Get the Input Capture value and calculate PWM frequency */
    data->rssi_frequency = RCINPUT_CAPTURE_TIMER_RATE / period;

    /* If there is valid data, save it else reset RSSI values */
    if (period != 0)
    {
        /* Convert the RSSI PWM to percent */
        data->rssi = (width * 100) / period;

        /* Check so the RSSI is above the threshold */
        if (data->rssi < RCINPUT_RSSI_THRESHOLD_PERCENT)
        {
            /* If the RSSI is below the threshold count until timeout */
            if (rssi_counter > RCINPUT_RSSI_TIMEOUT)
            {
                data->active_connection = FALSE;

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
        data->rssi = 0;
        data->rssi_frequency = 0;
    }
}

/**
 * @brief               Reset a RCInput data structure.
 *
 * @param[out] data     Pointer to RC Input data structure.
 */
static void RCInputDataReset(rcinput_data_t *data)
{
    int i;

    data->active_connection = FALSE;
    data->number_active_connections = 0;
    data->rssi = 0;
    data->rssi_frequency = 0;
    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
        data->value[i] = 0;
}

/**
 * @brief               Reset a RCInput data structure.
 *
 * @param[out] data     Pointer to RC Input data structure.
 */
static void RCInputSettingsReset(rcinput_settings_t *data)
{
    int i;

    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
    {
        data->role[i]      = ROLE_OFF;
        data->type[i]      = TYPE_ANALOG;
        data->ch_bottom[i] = 1000;
        data->ch_center[i] = 1500;
        data->ch_top[i]    = 2000;
    }

    data->mode = MODE_CPPM_INPUT;
}

/**
 * @brief               Converts role to corresponding array index.
 *
 * @param[in] role      Input role.
 * @return              Corresponding array index.
 */
static uint32_t RoleToIndex(input_role_selector_t role)
{
    /* Get the index of the associated role */
    if (role != ROLE_OFF)
        return ((role_lookup >> ((role - 1) * RCINPUT_ROLE_TO_INDEX_BITS)) &
                 RCINPUT_ROLE_TO_INDEX_MASK);
    else
        return RCINPUT_ROLE_TO_INDEX_MASK;
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

    rcinput_data.active_connection = FALSE;
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

    ParseCPPMInput(&rcinput_data, capture);

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

    ParseRSSIInput(&rcinput_data,
                   eicuGetWidth(eicup, EICU_CHANNEL_2),
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
    (void)eicup;
    (void)channel;
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
    RCInputDataReset(&rcinput_data);
    RCInputSettingsReset(&rcinput_settings);

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
    int i;

    /* If the EICU driver was already in use, disable it */
    if (EICUD3.state != EICU_STOP)
        eicuDisable(&EICUD3);
    if (EICUD9.state != EICU_STOP)
        eicuDisable(&EICUD9);
    if (EICUD12.state != EICU_STOP)
        eicuDisable(&EICUD12);

    /* Configure the input capture unit */
    if (rcinput_settings.mode == MODE_CPPM_INPUT)
    {
        /* Start and enable the EICU driver */
        eicuStart(&EICUD9, &cppm_rcinputcfg);
        eicuStart(&EICUD12, &rssi_rcinputcfg);
        eicuEnable(&EICUD9);
        eicuEnable(&EICUD12);
    }
    else if (rcinput_settings.mode == MODE_PWM_INPUT)
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

    /* For each role associated with a channel, generate
       the inverse lookup table */
    role_lookup = 0;

    for (i = 0; i < RCINPUT_MAX_NUMBER_OF_INPUTS; i++)
        if (rcinput_settings.role[i] != ROLE_OFF)
            role_lookup |= (i << ((rcinput_settings.role[i] - 1) *
                                   RCINPUT_ROLE_TO_INDEX_BITS));

    return status;
}

/**
 * @brief           Get the input level of the corresponding role and returns
 *                  it in the span of -1.0 to 1.0 or 0.0 to 1.0.
 *
 * @param[in] role  Input role for the RC input.
 * @return          The curernt input value of the corresponding role.
 */
float RCInputGetInputLevel(input_role_selector_t role)
{
    int32_t value, idx;
    float level;

    /* Get the position in the array for the requested role */
    idx = RoleToIndex(role);

    /* Check the validity of the index */
    if (idx == RCINPUT_ROLE_TO_INDEX_MASK)
        return 0.0f;

    /* Get the raw value from the raw data structure */
    value = rcinput_data.value[idx];

    /* Check so the data and input is valid */
    if ((value == 0) ||
        (role == ROLE_OFF) ||
        (rcinput_data.active_connection == FALSE))
        return 0.0f;

    /* Remove the center offset */
    level = (float)(value - rcinput_settings.ch_center[idx]);

    /* If it is larger than zero */
    if (level > 0.0f)
    {
        /* If the settings does not allow positive output */
        if (rcinput_settings.ch_center[idx] == rcinput_settings.ch_top[idx])
            return 0.0f;

        /* Use the calibration to calculate the position */
        level = level / (float)(rcinput_settings.ch_top[idx] -
                                rcinput_settings.ch_center[idx]);
    }
    /* If it is smaller than zero */
    else if (level < 0.0f)
    {
        /* If the settings does not allow negative output */
        if (rcinput_settings.ch_center[idx] == rcinput_settings.ch_bottom[idx])
            return 0.0f;

        /* Use the calibration to calculate the position */
        level = level / (float)(rcinput_settings.ch_center[idx] -
                                rcinput_settings.ch_bottom[idx]);
    }

    return bound(1.0f, -1.0f, level);
}

/**
 * @brief           Return the current connection status of the RC Input.
 *
 * @return          Return true if there is a connection, else false.
 */
bool bActiveRCInputConnection(void)
{
    if (rcinput_data.active_connection == FALSE)
        return false;
    else
        return true;
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
