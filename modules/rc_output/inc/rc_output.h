#ifndef __RC_OUTPUT_H
#define __RC_OUTPUT_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief Enum for selecting the rate of the output PPM.
 */
typedef enum
{
    RCOUTPUT_400HZ = 2500,
    RCOUTPUT_50HZ = 20000
} RCOutput_Rate_Selector;

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
} RCOutput_Channel_Selector;

/**
 * @brief Enum for defining which outputs belong to which PWM bank.
 */
typedef enum
{
    RCOUTPUT_BANK_1_4 = 0,
    RCOUTPUT_BANK_5_8 = 1
} RCOutput_Bank_Selector;

/**
 * @brief Configuration structure for RC ouput.
 */
typedef struct
{
    /**
     * @brief PWM driver for the low (1 to 4) bank.
     */
    PWMDriver *pwmp_lowbank;
    /**
     * @brief PWM driver for the high (5 to 8) bank.
     */
    PWMDriver *pwmp_highbank;
    /**
     * @brief PWM configuration structure.
     */
    const PWMConfig *pwmcfg;
} RCOutput_Configuration;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
msg_t RCOutputInit(const RCOutput_Configuration *cfg);
msg_t RCOutputSetChannelWidthUs(RCOutput_Channel_Selector sel,
                                pwmcnt_t width_us);
msg_t RCOutputSetChannelWidthRelativePositive(RCOutput_Channel_Selector sel,
                                              float width);
msg_t RCOutputSetChannelWidthRelative(RCOutput_Channel_Selector sel,
                                      float width);
msg_t RCOutputSetChannelPeriod(RCOutput_Bank_Selector sel,
                               RCOutput_Rate_Selector rate);


#endif
