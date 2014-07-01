#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

#include "circularbuffer.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define SYNC_BYTE                     (0xa6)
#define ACK_BIT                       (0x80)
#define SERIAL_RECIEVE_BUFFER_SIZE    (256)
#define SERIAL_TRANSMIT_BUFFER_SIZE   (1024)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Port type identifier for serial communication.
 */
typedef enum PACKED_VAR
{
    /**
     * @brief   USB identifier.
     */
    PORT_USB = 0,
    /**
     * @brief   AUX1 identifier.
     */
    PORT_AUX1 = 1,
    /**
     * @brief   AUX2 identifier.
     */
    PORT_AUX2 = 2,
    /**
     * @brief   AUX3 identifier.
     */
    PORT_AUX3 = 3,
    /**
     * @brief   AUX4 (CAN) identifier.
     */
    PORT_AUX4 = 4
} External_Port;

/**
 * @brief   All the commands for the serial protocol.
 */
typedef enum PACKED_VAR
{
    /**
     * @brief   The zero command is not allowed.
     */
    Cmd_None                        = 0,
    /**
     * @brief   ACK command.
     */
    Cmd_ACK                         = 1,
    /**
     * @brief   Ping command.
     */
    Cmd_Ping                        = 2,
    /**
     * @brief   Send debug message command.
     */
    Cmd_DebugMessage                = 3,
    /**
     * @brief   Get running mode command.
     */
    Cmd_GetRunningMode              = 4,
    /**
     * @brief   Manage Subscriptions command.
     */
    Cmd_ManageSubscriptions         = 5,
    /**
     * @brief   Custom command of experiments.
     */
    Cmd_GetExperimentData           = 6,


    /*===============================================*/
    /* Bootloader specific commands.                 */
    /*===============================================*/

    /**
     * @brief   Prepare to write firmware command.
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_PrepareWriteFirmware        = 10,
    /**
     * @brief   Write firmware package command.
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_WriteFirmwarePackage        = 11,
    /**
     * @brief   Write the last firmware package.
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_WriteLastFirmwarePackage    = 12,
    /**
     * @brief   Read firmware package.
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_ReadFirmwarePackage         = 13,
    /**
     * @brief   Read the last firmware package.
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_ReadLastFirmwarePackage     = 14,
    /**
     * @brief   Next package command (sent to PC to resume transmission).
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_NextPackage                 = 15,
    /**
     * @brief   Exit bootloader command.
     * @note    Bootloader specific, shall always require ACK.
     */
    Cmd_ExitBootloader              = 16,

    /*===============================================*/
    /* Info, ID and Save commands.                   */
    /*===============================================*/

    /**
     * @brief   Get device IDs.
     */
    Cmd_GetDeviceInfo               = 17,
    /**
     * @brief   Set the user definable ID string.
     */
    Cmd_SetDeviceID                 = 18,
    /**
     * @brief   Save all settings to flash.
     */
    Cmd_SaveToFlash                 = 19,

    /*===============================================*/
    /* Controller specific commands.                 */
    /*===============================================*/

    /**
     * @brief   Get Arm Settings.
     */
    Cmd_GetArmSettings              = 28,
    /**
     * @brief   Set Arm Settings.
     */
    Cmd_SetArmSettings              = 29,
    /**
     * @brief   Get rate controller data.
     */
    Cmd_GetRateControllerData       = 30,
    /**
     * @brief   Set rate controller data.
     */
    Cmd_SetRateControllerData       = 31,
    /**
     * @brief   Get attitude controller data.
     */
    Cmd_GetAttitudeControllerData   = 32,
    /**
     * @brief   Set attitude controller data.
     */
    Cmd_SetAttitudeControllerData   = 33,
    /**
     * @brief   Get velocity controller data.
     */
    Cmd_GetVelocityControllerData   = 34,
    /**
     * @brief   Set velocity controller data.
     */
    Cmd_SetVelocityControllerData   = 35,
    /**
     * @brief   Get position controller data.
     */
    Cmd_GetPositionControllerData   = 36,
    /**
     * @brief   Set position controller data.
     */
    Cmd_SetPositionControllerData   = 37,

    /*=======================================*/
    /* 38 is reserved! Command 38 + ACK will */
    /* become SYNC which is forbidden.       */
    /*=======================================*/

    /**
     * @brief   Get channel mixer.
     */
    Cmd_GetChannelMix               = 39,
    /**
     * @brief   Set channel mixer.
     */
    Cmd_SetChannelMix               = 40,

    /*===============================================*/
    /* RC specific commands.                         */
    /*===============================================*/

    /**
     * @brief   Get RC calibration.
     */
    Cmd_GetRCCalibration            = 41,
    /**
     * @brief   Set RC calibration.
     */
    Cmd_SetRCCalibration            = 42,
    /**
     * @brief   Get RC values.
     */
    Cmd_GetRCValues                 = 43,
    
    /*===============================================*/
    /* Sensor specific commands.                     */
    /*===============================================*/

    /**
     * @brief   Get calibrated sensor data.
     */
    Cmd_GetSensorData               = 44,
    /**
     * @brief   Get raw sensor data.
     */
    Cmd_GetRawSensorData            = 45,
    /**
     * @brief   Get sensor calibration.
     */
    Cmd_GetSensorCalibration        = 46,
    /**
     * @brief   Set sensor calibration.
     */
    Cmd_SetSensorCalibration        = 47,

    /*===============================================*/
    /* Estimation specific commands.                 */
    /*===============================================*/

    /**
     * @brief   Get rate estimation.
     */
    Cmd_GetEstimationRate           = 48,
    /**
     * @brief   Get attitude estimation.
     */
    Cmd_GetEstimationAttitude       = 49,
    /**
     * @brief   Get velocity estimation.
     */
    Cmd_GetEstimationVelocity       = 50,
    /**
     * @brief   Get position estimation.
     */
    Cmd_GetEstimationPosition       = 51,
    /**
     * @brief   Get all states.
     */
    Cmd_GetEstimationAllStates      = 52,
    /**
     * @brief   Reset estimation.
     */
    Cmd_ResetEstimation             = 53
} KFly_Command;

/**
 * @brief   The structure to keep track of transfers through the state machine.
 */
typedef struct _parser_holder
{
    /**
     * @brief   Which port the data came from.
     */
    External_Port Port;
    /**
     * @brief   If an ACK was requested.
     */
    bool AckRequested;
    /**
     * @brief   The length of the data.
     */
    uint8_t data_length;
    /**
     * @brief   Pointer to the buffer storing the data.
     */
    uint8_t *buffer;
    /**
     * @brief   The current location in the buffer.
     */
    uint16_t buffer_count;
    /**
     * @brief   The current CRC8 calculation.
     */
    uint8_t crc8;
    /**
     * @brief   The current CRC16 calculation.
     */
    uint16_t crc16;
    /**
     * @brief   The number of receive errors.
     */
    uint32_t rx_error;
    /**
     * @brief   The number of correctly received packets.
     */
    uint32_t rx_success;
    /**
     * @brief   Pointer to the current state in the state machine.
     */
    void (*current_state)(uint8_t, struct _parser_holder *);
    /**
     * @brief   Pointer to the next state in the state machine.
     */
    void (*next_state)(uint8_t, struct _parser_holder *);
    /**
     * @brief    Pointer to the parser to parse the data after a
     *           successful transfer.
     */
    void (*parser)(struct _parser_holder *);
} parser_holder_t;

/**
 * @brief   Function pointer definition for the Message Parser lookup table.
 */
typedef void (*parser_t)(struct _parser_holder *);

/**
 * @brief   Function pointer definition for the Message Generator lookup table.
 */
typedef bool (*generator_t)(circular_buffer_t *);

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

static inline bool isPort(External_Port port)
{
    if ((port == PORT_USB) ||
        (port == PORT_AUX1) ||
        (port == PORT_AUX2) ||
        (port == PORT_AUX3) ||
        (port == PORT_AUX4))
        return true;
    else
        return false;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void vInitStatemachineDataHolder(parser_holder_t *pHolder,
                                 External_Port port,
                                 uint8_t *buffer);
void vStatemachineDataEntry(uint8_t data, parser_holder_t *pHolder);
void CircularBuffer_WriteSYNCNoIncrement(circular_buffer_t *Cbuff, 
										 int32_t *count, 
										 uint8_t *crc8, 
										 uint16_t *crc16);
void CircularBuffer_WriteNoIncrement(circular_buffer_t *Cbuff,
                                     uint8_t data, 
                                     int32_t *count, 
                                     uint8_t *crc8, 
                                     uint16_t *crc16);

#endif
