#ifndef __RC_OUTPUT_H
#define __RC_OUTPUT_H

#include "ch.h"
#include "hal.h"
#include "kfly_defs.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define RCOUTPUT_SETTINGS_SIZE              (sizeof(rcoutput_settings_t))
#define RCOUTPUT_NUM_OUTPUTS                (8)
#define RCOUTPUT_BANK_SIZE                  (4)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,             // Needs to be sent 10x
    DSHOT_CMD_SPIN_DIRECTION_2,             // Needs to be sent 10x
    DSHOT_CMD_3D_MODE_OFF,                  // Needs to be sent 10x
    DSHOT_CMD_3D_MODE_ON,                   // Needs to be sent 10x
    DSHOT_CMD_SETTINGS_REQUEST,             // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,                // Needs to be sent 10x
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,   // Needs to be sent 10x
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21, // Needs to be sent 10x
    DSHOT_CMD_LED0_ON,                      // BLHeli32 only
    DSHOT_CMD_LED1_ON,                      // BLHeli32 only
    DSHOT_CMD_LED2_ON,                      // BLHeli32 only
    DSHOT_CMD_LED3_ON,                      // BLHeli32 only
    DSHOT_CMD_LED0_OFF,                     // BLHeli32 only
    DSHOT_CMD_LED1_OFF,                     // BLHeli32 only
    DSHOT_CMD_LED2_OFF,                     // BLHeli32 only
    DSHOT_CMD_LED3_OFF,                     // BLHeli32 only
    DSHOT_CMD_MAX = 47
} dshot_commands_t;

/**
 * @brief Enum for selecting the period of the output PPM.
 */
typedef enum
{
    RCOUTPUT_MODE_50HZ_PWM = 0,
    RCOUTPUT_MODE_400HZ_PWM,
    RCOUTPUT_MODE_ONESHOT125,
    RCOUTPUT_MODE_ONESHOT42,
    RCOUTPUT_MODE_MULTISHOT,
    RCOUTPUT_MODE_DSHOT150,
    RCOUTPUT_MODE_DSHOT300,
    RCOUTPUT_MODE_DSHOT600,
    RCOUTPUT_MODE_DSHOT1200,
    RCOUTPUT_MODE_ONESHOT_START = RCOUTPUT_MODE_ONESHOT125,
    RCOUTPUT_MODE_DSHOT_START = RCOUTPUT_MODE_DSHOT150
} rcoutput_mode_t;


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
 * @brief Configuration structure for RC output.
 */
typedef struct
{
    DMA_TypeDef *bank1_dmap;
    DMA_TypeDef *bank2_dmap;

    const stm32_dma_stream_t *bank1_dmasp;
    const stm32_dma_stream_t *bank2_dmasp;

    stm32_tim_t *bank1_tim;
    stm32_tim_t *bank2_tim;

    uint32_t bank1_tim_clock;
    uint32_t bank2_tim_clock;

    uint16_t bank1_buffer[RCOUTPUT_NUM_OUTPUTS * 2 + 1][RCOUTPUT_BANK_SIZE];
    uint16_t bank2_buffer[RCOUTPUT_NUM_OUTPUTS * 2 + 1][RCOUTPUT_BANK_SIZE];

    bool request_telemetry[RCOUTPUT_NUM_OUTPUTS];
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
/* External declarations.                                                    */
/*===========================================================================*/
void RCOutputInit(void);
void RCOutputTimerInit(void);
void RCOutputDisableI(void);
void RCOutputSetChannelWidth(const rcoutput_channel_t channel,
                             float value);
void RCOutputSync(void);
void vParseSetRCOutputSettings(const uint8_t *payload,
                               const size_t data_length);
rcoutput_settings_t *ptrGetRCOutoutSettings(void);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/


#endif
