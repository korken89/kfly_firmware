#ifndef __RC_OUTPUT_H
#define __RC_OUTPUT_H

/* Defines */
#define RCOUTPUT_1MHZ_CLOCK_FREQUENCY       1000000

/* Global variable defines */

/* Typedefs */
typedef enum
{
    RCOUTPUT_400HZ = 2500,
    RCOUTPUT_50HZ = 20000
} RCOutput_Rate_Selector;

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

typedef enum
{
    RCOUTPUT_BANK_1_4 = 0,
    RCOUTPUT_BANK_5_8 = 1
} RCOutput_Bank_Selector;

typedef struct
{
    PWMDriver *pwmp_lowbank;
    PWMDriver *pwmp_highbank;
    const PWMConfig *pwmcfg;
} RCOutput_Configuration;

/* Global function defines */
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
