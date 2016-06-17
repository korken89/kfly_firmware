/* *
 *
 * Abstraction Layer for RC Outputs
 *
 * */

#include "ch.h"
#include "hal.h"
#include "rc_output.h"
#include "flash_save.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief RC output PWM configuration.
 */
static const EPWMConfig pwmcfg = {
    RCOUTPUT_FREQUENCY_PWM,             /* 1 MHz PWM clock frequency    */
    RCOUTPUT_PERIOD_50HZ,               /* Initial PWM period: 50 Hz   */
    {
        {EPWM_OUTPUT_ACTIVE_HIGH},      /* Active high */
        {EPWM_OUTPUT_ACTIVE_HIGH},      /* Active high */
        {EPWM_OUTPUT_ACTIVE_HIGH},      /* Active high */
        {EPWM_OUTPUT_ACTIVE_HIGH}       /* Active high */
    },
    EPWM_PWM_MODE
};

/**
 * @brief RC output One Pulse Mode (OneShot125) configuration.
 */
static const EPWMConfig opmcfg = {
    RCOUTPUT_FREQUENCY_OPM,             /* 7 MHz clock frequency    */
    RCOUTPUT_PERIOD_OPM,
    {
        {EPWM_OUTPUT_ACTIVE_HIGH},      /* Active high */
        {EPWM_OUTPUT_ACTIVE_HIGH},      /* Active high */
        {EPWM_OUTPUT_ACTIVE_HIGH},      /* Active high */
        {EPWM_OUTPUT_ACTIVE_HIGH}       /* Active high */
    },
    EPWM_OPM_MODE
};

/**
 * @brief RC output configuration holder.
 */
static const rcoutput_configuration_t rcoutput_cfg = {
    &EPWMD4,
    &EPWMD8,
    &pwmcfg,
    &opmcfg
};

/**
 * @brief RC output channel lookup table for timer channel to output channel.
 */
static const epwmchannel_t rcoutput_channellut[] = {0, 1, 2, 3, 3, 2, 1, 0};

/**
 * @brief RC output bank settings.
 */
rcoutput_settings_t rcoutput_settings;

/**
 * @brief Working area for the RC output flash save thread.
 */
THD_WORKING_AREA(waThreadRCOutputFlashSave, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief           Thread for the flash save operation.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadRCOutputFlashSave, arg)
{
    UNUSED(arg);

    /* Event registration for new estimation */
    event_listener_t el;

    /* Set thread name */
    chRegSetThreadName("RCOutput FlashSave");

    /* Register to new estimation */
    chEvtRegisterMask(ptrGetFlashSaveEventSource(),
                      &el,
                      FLASHSAVE_SAVE_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation */
        chEvtWaitOne(FLASHSAVE_SAVE_EVENTMASK);

        /* Save RC input settings to flash */
        FlashSave_Write(FlashSave_STR2ID("RCOT"),
                        true,
                        (uint8_t *)&rcoutput_settings,
                        RCOUTPUT_SETTINGS_SIZE);
    }
}

/**
 * @brief   Reset for the RC output settings structure.
 */
static void RCOutputSettingsReset(void)
{
    int i;

    /* Set outputs to OneShot125 and enable all. */
    rcoutput_settings.mode_bank1 = RCOUTPUT_MODE_OPM;
    rcoutput_settings.mode_bank2 = RCOUTPUT_MODE_OPM;

    for (i = 0; i < RCOUTPUT_NUM_OUTPUTS; i++)
        rcoutput_settings.channel_enabled[i] = true;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes RC outputs' settings and data structures.
 */
void RCOutputInit(void)
{
    if ((rcoutput_cfg.pwmcfg == NULL)        ||
        (rcoutput_cfg.opmcfg == NULL)        ||
        (rcoutput_cfg.pwmp_highbank == NULL) ||
        (rcoutput_cfg.pwmp_lowbank == NULL))
        osalSysHalt("RC Output configuration error.");

    /* Reset the settings structure. */
    RCOutputSettingsReset();


    /* Read RC output settings from flash */
    FlashSave_Read(FlashSave_STR2ID("RCOT"),
                   (uint8_t *)&rcoutput_settings,
                   RCOUTPUT_SETTINGS_SIZE);

    /* Start the Flash Save thread */
    chThdCreateStatic(waThreadRCOutputFlashSave,
                      sizeof(waThreadRCOutputFlashSave),
                      NORMALPRIO,
                      ThreadRCOutputFlashSave,
                      NULL);

    /* Initialize outputs according to settings. */
    RCOutputInitialization();
}

/**
 * @brief   Initializes or reinitialized RC outputs based on the settings.
 */
void RCOutputInitialization(void)
{
    int i;

    /* If the output is already initialized, reinit. */
    if (rcoutput_settings.mode_bank1 == RCOUTPUT_MODE_OPM)
    {
        epwmStart(rcoutput_cfg.pwmp_lowbank, rcoutput_cfg.opmcfg);
    }
    else
    {
        epwmStart(rcoutput_cfg.pwmp_lowbank, rcoutput_cfg.pwmcfg);

        /* If the it is set to 400 Hz PWM, update the period. */
        if (rcoutput_settings.mode_bank1 == RCOUTPUT_MODE_400HZ_PWM)
            epwmChangePeriod(rcoutput_cfg.pwmp_lowbank, RCOUTPUT_PERIOD_400HZ);

        /* Initialize with lowest output width */
        for (i = 0; i < RCOUTPUT_BANK_SIZE; i++)
            if (rcoutput_settings.channel_enabled[i] == true)
                epwmEnableChannel(rcoutput_cfg.pwmp_lowbank,
                                  rcoutput_channellut[i],
                                  0);
    }

    if (rcoutput_settings.mode_bank2 == RCOUTPUT_MODE_OPM)
    {
        epwmStart(rcoutput_cfg.pwmp_highbank, rcoutput_cfg.opmcfg);
    }
    else
    {
        epwmStart(rcoutput_cfg.pwmp_highbank, rcoutput_cfg.pwmcfg);

        /* If the it is set to 400 Hz PWM, update the period. */
        if (rcoutput_settings.mode_bank2 == RCOUTPUT_MODE_400HZ_PWM)
            epwmChangePeriod(rcoutput_cfg.pwmp_highbank, RCOUTPUT_PERIOD_400HZ);

        /* Initialize with lowest output width */
        for (i = RCOUTPUT_BANK_SIZE; i < 2*RCOUTPUT_BANK_SIZE; i++)
            if (rcoutput_settings.channel_enabled[i] == true)
                epwmEnableChannel(rcoutput_cfg.pwmp_highbank,
                                  rcoutput_channellut[i],
                                  0);
    }
}

/**
 * @brief   Disables all outputs, used when force disable of outputs is needed.
 */
void RCOutputDisableI(void)
{
    epwmDisableChannelI(rcoutput_cfg.pwmp_lowbank, 0);
    epwmDisableChannelI(rcoutput_cfg.pwmp_lowbank, 1);
    epwmDisableChannelI(rcoutput_cfg.pwmp_lowbank, 2);
    epwmDisableChannelI(rcoutput_cfg.pwmp_lowbank, 3);

    epwmDisableChannelI(rcoutput_cfg.pwmp_highbank, 0);
    epwmDisableChannelI(rcoutput_cfg.pwmp_highbank, 1);
    epwmDisableChannelI(rcoutput_cfg.pwmp_highbank, 2);
    epwmDisableChannelI(rcoutput_cfg.pwmp_highbank, 3);
}

/**
 * @brief               Set output channel width in microseconds.
 *
 * @param[in] sel       Channel selector.
 * @param[in] width     New width in the timer value.
 * @return              MSG_OK if the change was successful.
 */
msg_t RCOutputSetChannelWidthAbsolute(const rcoutput_channel_t sel,
                                      const epwmcnt_t width)
{

    if (sel > RCOUTPUT_CHANNEL_8)
        return MSG_RESET;
    else
    {
        if (sel <= RCOUTPUT_CHANNEL_4)
            epwmEnableChannel(rcoutput_cfg.pwmp_lowbank,
                              rcoutput_channellut[sel],
                              width);
        else
            epwmEnableChannel(rcoutput_cfg.pwmp_highbank,
                              rcoutput_channellut[sel],
                              width);

        return MSG_OK;
    }
}

/**
 * @brief               Set relative output channel width
 * @details             Set the width using 0% to 100%.
 *
 * @param[in] sel       Channel selector.
 * @param[in] width     New width in 0.0 to 1.0.
 * @return              MSG_OK if the change was successful.
 */
msg_t RCOutputSetChannelWidth(const rcoutput_channel_t sel,
                              float width)
{
    if (sel > RCOUTPUT_CHANNEL_8)
        return MSG_RESET;
    else if (rcoutput_settings.channel_enabled[sel] == false)
        return MSG_RESET;
    else
    {
        /* Bound the width from 0 % to 100 % */
        if (width < 0.0f)
            width = 0.0f;
        else if (width > 1.0f)
            width = 1.0f;

        /* Convert to absolute and send */
        if ((sel <= RCOUTPUT_CHANNEL_4) &&
            (rcoutput_settings.mode_bank1 == RCOUTPUT_MODE_OPM))
        {
            return RCOutputSetChannelWidthAbsolute(sel,
                (epwmcnt_t)(width * RCOUTPUT_OPM_WIDTH + RCOUTPUT_OPM_WIDTH));
        }
        else if ((sel > RCOUTPUT_CHANNEL_4) &&
                 (rcoutput_settings.mode_bank2 == RCOUTPUT_MODE_OPM))
        {
            return RCOutputSetChannelWidthAbsolute(sel,
                (epwmcnt_t)(width * RCOUTPUT_OPM_WIDTH + RCOUTPUT_OPM_WIDTH));
        }
        else
        {
            return RCOutputSetChannelWidthAbsolute(sel,
                (epwmcnt_t)(width * RCOUTPUT_PWM_WIDTH + RCOUTPUT_PWM_WIDTH));
        }
    }
}

/**
 * @brief   Syncs the output (sends the pulses) when in OPM mode.
 */
void RCOutputSyncOutput(void)
{
    if (rcoutput_settings.mode_bank1 == RCOUTPUT_MODE_OPM)
        epwmSendPulses(rcoutput_cfg.pwmp_lowbank);
    if (rcoutput_settings.mode_bank2 == RCOUTPUT_MODE_OPM)
        epwmSendPulses(rcoutput_cfg.pwmp_highbank);
}

/**
 * @brief               Parses a payload from the serial communication for
 *                      all the RC output settings.
 *
 * @param[in] payload   Pointer to the payload location.
 * @param[in] size      Size of the payload.
 */
void vParseSetRCOutputSettings(const uint8_t *payload,
                               const size_t data_length)
{
    if (data_length == RCOUTPUT_SETTINGS_SIZE)
    {
        osalSysLock();

        /* Save the data */
        memcpy((uint8_t *)ptrGetRCOutoutSettings(),
               payload,
               RCOUTPUT_SETTINGS_SIZE);

        osalSysUnlock();
    }

    /* Reinitialize the RC Output module. */
    RCOutputInitialization();
}

/**
 * @brief           Return the pointer to the RC Input settings.
 *
 * @return          Pointer to the RC input settings.
 */
rcoutput_settings_t *ptrGetRCOutoutSettings(void)
{
    return &rcoutput_settings;
}
