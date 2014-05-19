/* *
 *
 * Abstraction Layer for RC Outputs
 *
 * */

#include "ch.h"
#include "hal.h"
#include "eicu.h"
#include "rc_input.h"

/* Global variable defines */

/* Private function defines */
static void ParseCPPMInput(RCInput_Data *data,
                           uint32_t capture);
static void ParseRSSIInput(RCInput_Data *data,
                           uint32_t width,
                           uint32_t period);
static void RCInput_data_reset(RCInput_Data *data);
static void vt_no_connection_timeout_callback(void *p);
static void cppm_callback(EICUDriver *eicup, eicuchannel_t channel);
static void rssi_callback(EICUDriver *eicup, eicuchannel_t channel);
static void pwm_callback(EICUDriver *eicup, eicuchannel_t channel);

/* Private variable defines */

CCM_MEMORY static RCInput_Data rcinputdata;
static event_source_t rcinput_es;
static virtual_timer_t rcinput_timeout_vt;

/* EICU Configuration for CPPM, RSSI and PWM inputs */
static const EICU_IC_Settings cppmsettings = {
    EICU_INPUT_ACTIVE_LOW,
    cppm_callback
};
static const EICUConfig cppm_rcinputcfg = {
    EICU_INPUT_EDGE,
    CAPTURE_TIMER_RATE,
    {
        &cppmsettings,
        NULL,
        NULL,
        NULL
    },
    NULL,
    NULL,
    0,
    0
};

static const EICU_IC_Settings rssisettings = {
    EICU_INPUT_ACTIVE_HIGH,
    NULL
};
static const EICUConfig rssi_rcinputcfg = {
    EICU_INPUT_PWM,
    CAPTURE_TIMER_RATE,
    {
        NULL,
        &rssisettings,
        NULL,
        NULL
    },
    rssi_callback,
    NULL,
    EICU_PWM_CHANNEL_2,
    0
};

static const EICU_IC_Settings pwmsettings = {
    EICU_INPUT_ACTIVE_HIGH,
    pwm_callback
};
static const EICUConfig pwm_rcinputcfg_1 = {
    EICU_INPUT_PULSE,
    CAPTURE_TIMER_RATE,
    {
        &pwmsettings,
        &pwmsettings,
        NULL,
        NULL
    },
    NULL,
    NULL,
    0,
    0
};

static const EICUConfig pwm_rcinputcfg_2 = {
    EICU_INPUT_PULSE,
    CAPTURE_TIMER_RATE,
    {
        NULL,
        NULL,
        &pwmsettings,
        &pwmsettings
    },
    NULL,
    NULL,
    0,
    0
};



/* Private external functions */

/**
 * @brief           Initializes RC inputs.
 * 
 * @param[in] mode  Input mode for the RC inputs.
 * @return          MSG_OK if the initialization was successful.
 */
msg_t RCInputInit(RCInput_Mode_Selector mode)
{
    msg_t status = MSG_OK;

    /* Initialize the RC Input event source */
    osalEventObjectInit(&rcinput_es);

    /* Initialize data structure */
    RCInput_data_reset(&rcinputdata);

    /* Configure the input capture unit */
    if (mode == MODE_CPPM_INPUT)
    {
        if (EICUD9.state != EICU_STOP)
            eicuDisable(&EICUD9);
        if (EICUD12.state != EICU_STOP)
            eicuDisable(&EICUD12);
        eicuStart(&EICUD9, &cppm_rcinputcfg);
        eicuStart(&EICUD12, &rssi_rcinputcfg);
        eicuEnable(&EICUD9);
        eicuEnable(&EICUD12);
    }
    else /* PWM input */
    {
        if (EICUD3.state != EICU_STOP)
            eicuDisable(&EICUD3);
        if (EICUD9.state != EICU_STOP)
            eicuDisable(&EICUD9);
        if (EICUD12.state != EICU_STOP)
            eicuDisable(&EICUD12);
        eicuStart(&EICUD3, &pwm_rcinputcfg_2);
        eicuStart(&EICUD9, &pwm_rcinputcfg_1);
        eicuStart(&EICUD12, &pwm_rcinputcfg_1);
        eicuEnable(&EICUD3);
        eicuEnable(&EICUD9);
        eicuEnable(&EICUD12);
    }

    osalSysLock();

    chVTSetI(&rcinput_timeout_vt,
             MS2ST(RCINPUT_NO_CON_TIMEOUT_MS),
             vt_no_connection_timeout_callback,
             NULL);

    osalSysUnlock();

    return status;
}

/**
 * @brief           Pointer to the RC Input event source.
 * 
 * @return          returns the pointer to the RC input event source.
 */
event_source_t *ptrGetRCInputEventSource(void)
{
    return &rcinput_es;
}

static uint16_t rssi_counter = 0;

/**
 * @brief               Parses CPPM inputs.
 * 
 * @param[out] data     Pointer to RC Input data structure.
 * @param[in] capture   The value of the latest input capture.
 */
static void ParseCPPMInput(RCInput_Data *data,
                           uint32_t capture)
{
    static uint16_t cppm_count = 0; /* Current CPPM channel */

    if (data->active_connection == TRUE)
    {   
        /* If the capture is larger than the time for the SYNC, reset counter */
        if (capture > CPPM_SYNC_LIMIT_MIN)
        {
            /* Save the current number of active channels */
            data->number_active_connections = cppm_count;

            /* Reset CPPM counter */
            cppm_count = 0;

            /* Reset timeout and broadcast new input */
            osalSysLockFromISR();

            chVTSetI(&rcinput_timeout_vt,
                     MS2ST(RCINPUT_NO_CON_TIMEOUT_MS),
                     vt_no_connection_timeout_callback,
                     NULL);

            if (chEvtIsListeningI(&rcinput_es))
                chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_NEWINPUT_EVENTMASK);

            osalSysUnlockFromISR();
        }
        else
        {
            /* If no sync has been detected */
            if (cppm_count >= MAX_NUMBER_OF_INPUTS)
            {
                /* Reset connection */
                data->active_connection = FALSE;

                /*Disable timeout timer and broadcast connection lost */
                osalSysLockFromISR();

                chVTResetI(&rcinput_timeout_vt);

                if (chEvtIsListeningI(&rcinput_es))
                    chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_LOST_EVENTMASK);

                osalSysUnlockFromISR();
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
    else if ((capture > CPPM_SYNC_LIMIT_MIN) && \
             (capture < CPPM_SYNC_LIMIT_MAX) && \
             (rssi_counter < RSSI_TIMEOUT))
    {
        /* Sync found, reset CPPM counter and activate connection */
        cppm_count = 0;
        data->active_connection = TRUE;

        /* Enable timeout timer and broadcast connection active */
        osalSysLockFromISR();

        chVTSetI(&rcinput_timeout_vt,
                 MS2ST(RCINPUT_NO_CON_TIMEOUT_MS),
                 vt_no_connection_timeout_callback,
                 NULL);

        if (chEvtIsListeningI(&rcinput_es))
            chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_ACTIVE_EVENTMASK);
        
        osalSysUnlockFromISR();
    }
}

/**
 * @brief               Parses RSSI inputs.
 * 
 * @param[out] data     Pointer to RC Input data structure.
 * @param[in] width     The value of the latest width capture.
 * @param[in] period    The value of the latest period capture.
 */
static void ParseRSSIInput(RCInput_Data *data,
                           uint32_t width,
                           uint32_t period)
{
    /* Get the Input Capture value and calculate PWM frequency */
    data->rssi_frequency = CAPTURE_TIMER_RATE / period;

    /* If there is valid data, save it else reset RSSI values */
    if (period != 0)
    {
        /* Convert the RSSI PWM to percent */
        data->rssi = (width * 100) / period;

        /* Check so the RSSI is above the threshold */
        if (data->rssi < RSSI_THRESHOLD_PERCENT)
        {
            /* If the RSSI is below the threshold count until timeout */
            if (rssi_counter > RSSI_TIMEOUT)
            {
                data->active_connection = FALSE;

                /* Disable timeout timer and broadcast connection lost */
                osalSysLockFromISR();

                chVTResetI(&rcinput_timeout_vt);

                if (chEvtIsListeningI(&rcinput_es))
                    chEvtBroadcastFlagsI(&rcinput_es, RCINPUT_LOST_EVENTMASK);

                osalSysUnlockFromISR();
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
static void RCInput_data_reset(RCInput_Data *data)
{
    int i;

    data->active_connection = FALSE;
    data->number_active_connections = 0;
    data->rssi = 0;
    data->rssi_frequency = 0;
    for (i = 0; i < MAX_NUMBER_OF_INPUTS; i++)
        data->value[i] = 0;
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

    rcinputdata.active_connection = FALSE;
    chVTResetI(&rcinput_timeout_vt);

    if (chEvtIsListeningI(&rcinput_es))
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
    uint32_t capture = eicuGetWidth(eicup, channel);
    uint32_t last_count = eicup->last_count[0];
    eicup->last_count[0] = capture;

    if (capture > last_count)       /* No overflow */
        capture = capture - last_count;
    else if (capture < last_count)  /* Timer overflow */
        capture = ((0xFFFF - last_count) + capture); 

    ParseCPPMInput(&rcinputdata,
                   capture);
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
    ParseRSSIInput(&rcinputdata,
                   eicuGetWidth(eicup, EICU_PWM_CHANNEL_2),
                   eicuGetPeriod(eicup));
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
    //ic_test = eicuGetWidth(eicup, channel);
    palTogglePad(GPIOC, GPIOC_LED_ERR);
}
