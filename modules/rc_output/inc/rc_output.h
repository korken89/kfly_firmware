#ifndef __RC_OUTPUT_H
#define __RC_OUTPUT_H

#include "kfly_defs.h"
#include "epwm.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define RCOUTPUT_SETTINGS_SIZE              (sizeof(rcoutput_settings_t))
#define RCOUTPUT_NUM_OUTPUTS                (8)
#define RCOUTPUT_BANK_SIZE                  (4)
#define RCOUTPUT_OPM_WIDTH                  (875.0f)
#define RCOUTPUT_PWM_WIDTH                  (1000.0f)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Enum for selecting the period of the output PPM.
 */
typedef enum
{
    RCOUTPUT_MODE_400HZ_PWM = 0,
    RCOUTPUT_MODE_50HZ_PWM = 1,
    RCOUTPUT_MODE_OPM = 2
} rcoutput_mode_t;

/**
 * @brief Enum for selecting the period of the output PPM.
 */
typedef enum
{
    RCOUTPUT_PERIOD_400HZ = 2500,
    RCOUTPUT_PERIOD_50HZ = 20000,
    RCOUTPUT_PERIOD_OPM = 1760   /* 10 ticks of padding added. */
} rcoutput_period_t;

/**
 * @brief Enum for selecting the frequency of the timer.
 */
typedef enum
{
    RCOUTPUT_FREQUENCY_PWM = 1000000,
    RCOUTPUT_FREQUENCY_OPM = 7000000
} rcoutput_frequency_t;

/**
 * @brief Enum for defining the RC output channels.
 */
typedef enum
{
    RCOUTPUT_CHANNEL_1 = 0,
    RCOUTPUT_CHANNEL_2 = 1,
    RCOUTPUT_CHANNEL_3 = 2,
    RCOUTPUT_CHANNEL_4 = 3,
    RCOUTPUT_CHANNEL_5 = 4,
    RCOUTPUT_CHANNEL_6 = 5,
    RCOUTPUT_CHANNEL_7 = 6,
    RCOUTPUT_CHANNEL_8 = 7
} rcoutput_channel_t;

/**
 * @brief Enum for defining which outputs belong to which PWM bank.
 */
typedef enum
{
    RCOUTPUT_BANK_1_4 = 0,
    RCOUTPUT_BANK_5_8 = 1
} rcoutput_bank_t;

/**
 * @brief Configuration structure for RC ouput.
 */
typedef struct
{
    /**
     * @brief PWM driver for the low (1 to 4) bank.
     */
    EPWMDriver *pwmp_lowbank;
    /**
     * @brief PWM driver for the high (5 to 8) bank.
     */
    EPWMDriver *pwmp_highbank;
    /**
     * @brief PWM configuration structure.
     */
    const EPWMConfig *pwmcfg;
    /**
     * @brief PWM configuration structure.
     */
    const EPWMConfig *opmcfg;
} rcoutput_configuration_t;

/**
 * @brief RC ouput settings holder.
 */
typedef struct PACKED_VAR {
    /**
     * @brief Functional state of bank 1.
     */
    rcoutput_mode_t mode_bank1;
    /**
     * @brief Functional state of bank 2.
     */
    rcoutput_mode_t mode_bank2;
    /**
     * @brief Switch for enabling a channel.
     */
    bool channel_enabled[RCOUTPUT_NUM_OUTPUTS];
} rcoutput_settings_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void RCOutputInit(void);
void RCOutputInitialization(void);
void RCOutputDisableI(void);
msg_t RCOutputSetChannelWidthAbsolute(const rcoutput_channel_t sel,
                                      const epwmcnt_t width);
msg_t RCOutputSetChannelWidth(const rcoutput_channel_t sel,
                              const float width);
msg_t RCOutputSetChannelPeriod(const rcoutput_bank_t sel,
                               const rcoutput_period_t rate);
void RCOutputSyncOutput(void);
void vParseSetRCOutputSettings(const uint8_t *payload,
                               const size_t data_length);
rcoutput_settings_t *ptrGetRCOutoutSettings(void);

#endif
