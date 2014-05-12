#ifndef __STATEMACHINE_TYPES_H
#define __STATEMACHINE_TYPES_H

#define SYNC_BYTE               (0xa6)
#define ACK_BIT                 (0x80)
#define SERIAL_BUFFER_SIZE      (256)

/* These are all the commands for the serial protocol. */
typedef enum
{
    Cmd_None                        = 0,
    Cmd_ACK                         = 1,
    Cmd_Ping                        = 2,
    Cmd_DebugMessage                = 3,
    Cmd_GetRunningMode              = 4,

    Cmd_PrepareWriteFirmware        = 10,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_WriteFirmwarePackage        = 11,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_WriteLastFirmwarePackage    = 12,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_ReadFirmwarePackage         = 13,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_ReadLastFirmwarePackage     = 14,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_NextPackage                 = 15,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_ExitBootloader              = 16,   /* Bootloader specific, 
                                               shall always require ACK */
    Cmd_GetDeviceInfo               = 17,
    Cmd_SetDeviceID                 = 18,
    Cmd_SaveToFlash                 = 19,

    Cmd_GetRateControllerData       = 30,
    Cmd_SetRateControllerData       = 31,
    Cmd_GetAttitudeControllerData   = 32,
    Cmd_SetAttitudeControllerData   = 33,
    Cmd_GetVelocityControllerData   = 34,
    Cmd_SetVelocityControllerData   = 35,
    Cmd_GetPositionControllerData   = 36,
    Cmd_SetPositionControllerData   = 37,
    /* 38 is reserved! Command 38 + ACK will become SYNC which is forbidden. */
    Cmd_GetChannelMix               = 39,
    Cmd_SetChannelMix               = 40,
    Cmd_GetRCCalibration            = 41,
    Cmd_SetRCCalibration            = 42,
    Cmd_GetRCValues                 = 43,
    Cmd_GetSensorData               = 44,
    Cmd_GetRawSensorData            = 45,
    Cmd_GetSensorCalibration        = 46,
    Cmd_SetSensorCalibration        = 47,
    Cmd_GetEstimationRate           = 48,
    Cmd_GetEstimationAttitude       = 49,
    Cmd_GetEstimationVelocity       = 50,
    Cmd_GetEstimationPosition       = 51,
    Cmd_GetEstimationAllStates      = 52,
    Cmd_ResetEstimation             = 53
} KFly_Command_Type;

/* The structure to keep track of transfers through the state machine */
typedef struct _parser_holder
{
    Port_Type Port;                                         /* Which port the
                                                               data came from */
    Bool AckRequested;                                      /* If an ACK was
                                                               requested      */
    uint8_t data_length;                                    /* The length of 
                                                               the data       */
    uint8_t *buffer;                                        /* Pointer to the 
                                                               buffer storing 
                                                               the data       */
    uint16_t buffer_count;                                  /* The current 
                                                               location in the 
                                                               buffer         */
    uint8_t crc8;                                           /* The current CRC8 
                                                               calculation    */
    uint16_t crc16;                                         /* The current CRC16
                                                               calculation    */
    uint32_t rx_error;                                      /* The number of 
                                                               receive errors */
    void (*current_state)(uint8_t, struct _parser_holder *);/* Current state 
                                                               in the state 
                                                               machine        */
    void (*next_state)(uint8_t, struct _parser_holder *);   /* Next state in 
                                                               the state
                                                               machine        */
    void (*parser)(struct _parser_holder *);                /* Parser to parse 
                                                               the data after a 
                                                               successful 
                                                               transfer       */
} Parser_Holder_Type;

/* Function pointer definition for the Message Parser lookup table */
typedef void (*Parser_Type)(struct _parser_holder *);

/* Function pointer definition for the Message Generator lookup table */
typedef ErrorStatus (*Generator_Type)(Circular_Buffer_Type *);


#endif
