/* *
 *
 * All the generators for the different messages.
 * 
 * */

#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "version_information.h"
#include "serialmanager_types.h"
#include "serialmanager.h"
#include "crc.h"
//#include "pid.h"
//#include "sensor_read.h"
//#include "sensor_calibration.h"
//#include "control.h"
//#include "estimation.h"
//#include "ext_input.h"
#include "statemachine_parsers.h"
#include "statemachine_generators.h"

/* Private functions */
bool GenerateGenericGetControllerData(KFly_Command_Type command,
                                      const uint32_t pi_offset, 
                                      const uint32_t limit_offset, 
                                      const uint32_t limit_count, 
                                      Circular_Buffer_Type *Cbuff);

/**
 * The message generation lookup table
 */
static const Generator_Type generator_lookup[128] = {
    NULL,                             /* 0:   Cmd_None                        */
    GenerateACK,                      /* 1:   Cmd_ACK                         */
    GeneratePing,                     /* 2:   Cmd_Ping                        */
    NULL,                             /* 3:   Cmd_DebugMessage                */
    GenerateGetRunningMode,           /* 4:   Cmd_GetRunningMode              */
    NULL,                             /* 5:                                   */
    NULL,                             /* 6:                                   */
    NULL,                             /* 7:                                   */
    NULL,                             /* 8:                                   */
    NULL,                             /* 9:                                   */
    NULL,                             /* 10:  Cmd_PrepareWriteFirmware        */
    NULL,                             /* 11:  Cmd_WriteFirmwarePackage        */
    NULL,                             /* 12:  Cmd_WriteLastFirmwarePackage    */
    NULL,                             /* 13:  Cmd_ReadFirmwarePackage         */
    NULL,                             /* 14:  Cmd_ReadLastFirmwarePackage     */
    NULL,                             /* 15:  Cmd_NextPackage                 */
    NULL,                             /* 16:  Cmd_ExitBootloader              */
    GenerateGetDeviceInfo,            /* 17:  Cmd_GetDeviceInfo               */
    NULL,                             /* 18:  Cmd_SetDeviceID                 */
    NULL,                             /* 19:  Cmd_SaveToFlash                 */
    NULL,                             /* 20:                                  */
    NULL,                             /* 21:                                  */
    NULL,                             /* 22:                                  */
    NULL,                             /* 23:                                  */
    NULL,                             /* 24:                                  */
    NULL,                             /* 25:                                  */
    NULL,                             /* 26:                                  */
    NULL,                             /* 27:                                  */
    NULL,                             /* 28:                                  */
    NULL,                             /* 29:                                  */
    GenerateGetRateControllerData,    /* 30:  Cmd_GetRateControllerData       */
    NULL,                             /* 31:  Cmd_SetRateControllerData       */
    GenerateGetAttitudeControllerData,/* 32:  Cmd_GetAttitudeControllerData   */
    NULL,                             /* 33:  Cmd_SetAttitudeControllerData   */
    GenerateGetVelocityControllerData,/* 34:  Cmd_GetVelocityControllerData   */
    NULL,                             /* 35:  Cmd_SetVelocityControllerData   */
    GenerateGetPositionControllerData,/* 36:  Cmd_GetPositionControllerData   */
    NULL,                             /* 37:  Cmd_SetPositionControllerData   */
    NULL,                             /* 38:  RESERVED                        */
    NULL,                             /* 39:  Cmd_GetChannelMix               */
    NULL,                             /* 40:  Cmd_SetChannelMix               */
    GenerateGetRCCalibration,         /* 41:  Cmd_GetRCCalibration            */
    NULL,                             /* 42:  Cmd_SetRCCalibration            */
    GenerateGetRCValues,              /* 43:  Cmd_GetRCValues                 */
    GenerateGetSensorData,            /* 44:  Cmd_GetSensorData               */
    GenerateGetRawSensorData,         /* 45:  Cmd_GetRawSensorData            */
    GenerateGetSensorCalibration,     /* 46:  Cmd_GetSensorCalibration        */
    NULL,                             /* 47:                                  */
    GenerateGetEstimationRate,        /* 48:  Cmd_GetEstimationRate           */
    GenerateGetEstimationAttitude,    /* 49:  Cmd_GetEstimationAttitude       */
    GenerateGetEstimationVelocity,    /* 50:  Cmd_GetEstimationVelocity       */
    GenerateGetEstimationPosition,    /* 51:  Cmd_GetEstimationPosition       */
    GenerateGetEstimationAllStates,   /* 52:  Cmd_GetEstimationAllStates      */
    NULL,                             /* 53:                                  */
    NULL,                             /* 54:                                  */
    NULL,                             /* 55:                                  */
    NULL,                             /* 56:                                  */
    NULL,                             /* 57:                                  */
    NULL,                             /* 58:                                  */
    NULL,                             /* 59:                                  */
    NULL,                             /* 60:                                  */
    NULL,                             /* 61:                                  */
    NULL,                             /* 62:                                  */
    NULL,                             /* 63:                                  */
    NULL,                             /* 64:                                  */
    NULL,                             /* 65:                                  */
    NULL,                             /* 66:                                  */
    NULL,                             /* 67:                                  */
    NULL,                             /* 68:                                  */
    NULL,                             /* 69:                                  */
    NULL,                             /* 70:                                  */
    NULL,                             /* 71:                                  */
    NULL,                             /* 72:                                  */
    NULL,                             /* 73:                                  */
    NULL,                             /* 74:                                  */
    NULL,                             /* 75:                                  */
    NULL,                             /* 76:                                  */
    NULL,                             /* 77:                                  */
    NULL,                             /* 78:                                  */
    NULL,                             /* 79:                                  */
    NULL,                             /* 80:                                  */
    NULL,                             /* 81:                                  */
    NULL,                             /* 82:                                  */
    NULL,                             /* 83:                                  */
    NULL,                             /* 84:                                  */
    NULL,                             /* 85:                                  */
    NULL,                             /* 86:                                  */
    NULL,                             /* 87:                                  */
    NULL,                             /* 88:                                  */
    NULL,                             /* 89:                                  */
    NULL,                             /* 90:                                  */
    NULL,                             /* 91:                                  */
    NULL,                             /* 92:                                  */
    NULL,                             /* 93:                                  */
    NULL,                             /* 94:                                  */
    NULL,                             /* 95:                                  */
    NULL,                             /* 96:                                  */
    NULL,                             /* 97:                                  */
    NULL,                             /* 98:                                  */
    NULL,                             /* 99:                                  */
    NULL,                             /* 100:                                 */
    NULL,                             /* 101:                                 */
    NULL,                             /* 102:                                 */
    NULL,                             /* 103:                                 */
    NULL,                             /* 104:                                 */
    NULL,                             /* 105:                                 */
    NULL,                             /* 106:                                 */
    NULL,                             /* 107:                                 */
    NULL,                             /* 108:                                 */
    NULL,                             /* 109:                                 */
    NULL,                             /* 110:                                 */
    NULL,                             /* 111:                                 */
    NULL,                             /* 112:                                 */
    NULL,                             /* 113:                                 */
    NULL,                             /* 114:                                 */
    NULL,                             /* 115:                                 */
    NULL,                             /* 116:                                 */
    NULL,                             /* 117:                                 */
    NULL,                             /* 118:                                 */
    NULL,                             /* 119:                                 */
    NULL,                             /* 120:                                 */
    NULL,                             /* 121:                                 */
    NULL,                             /* 122:                                 */
    NULL,                             /* 123:                                 */
    NULL,                             /* 124:                                 */
    NULL,                             /* 125:                                 */
    NULL,                             /* 126:                                 */
    NULL                              /* 127:                                 */
};

 /**
  * @brief              Generate a message for the ports based on the
  *                     generators in the lookup table.
  * 
  * @param[in] command  The command to generate a message for.
  * @param[in] port     Which port to send the data.
  * @return             HAL_FAILED if the message didn't fit or HAL_SUCCESS
  *                     if it did fit.
  */
bool GenerateMessage(KFly_Command_Type command, Port_Type port)
{
    (void)port;

    bool status;
    Circular_Buffer_Type *Cbuff = NULL;

    Cbuff = SerialManager_GetCircularBufferFromPort(port);

    /* Check so the circular buffer address is valid */
    if (Cbuff == NULL)
        return HAL_FAILED;

    /* Check so there is an available Generator function for this command */
    if (generator_lookup[command] != NULL)
    {
        /* Claim the circular buffer for writing */
        CircularBuffer_Claim(Cbuff);
        {
            status = generator_lookup[command](Cbuff);
        }
        /* Release the circular buffer */
        CircularBuffer_Release(Cbuff);

        /* If it was successful then start the transmission */
        if (status == HAL_SUCCESS)
            SerialManager_StartTransmission(port);
    }
    else
        status = HAL_FAILED;
    
    return status;
}

/**
 * @brief               Generates a message with no data part.
 * 
 * @param[in] command   Command to generate message for.
 * @param[in] Cbuff     Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS 
 *                      if it did fit.
 */
bool GenerateHeaderOnlyCommand(KFly_Command_Type command, 
                               Circular_Buffer_Type *Cbuff)
{
    int32_t count = 0;
    uint8_t crc8;

    /* Write the stating SYNC (without doubling it) */
    CircularBuffer_WriteSYNCNoIncrement(Cbuff, &count, &crc8, NULL); 

    /* Add all the data to the message */
    CircularBuffer_WriteNoIncrement(Cbuff, command, &count, &crc8, NULL); 
    CircularBuffer_WriteNoIncrement(Cbuff, 0,       &count, &crc8, NULL); 
    CircularBuffer_WriteNoIncrement(Cbuff, crc8,    &count, NULL,  NULL);

    /* Check if the message fit inside the buffer */
    return CircularBuffer_Increment(Cbuff, count); 
}

/**
 * @brief                   Generates a message with data and CRC16 part. 
 * 
 * @param[in] command       Command to generate message for.
 * @param[in] data          Pointer to where the data is located.
 * @param[in] data_count    Number of data bytes.
 * @param[out] Cbuff        Pointer to the circular buffer to put the data in.
 * @return                  HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                          if it did fit.
 */
bool GenerateGenericCommand(KFly_Command_Type command, 
                            uint8_t *data, 
                            const uint32_t data_count, 
                            Circular_Buffer_Type *Cbuff)
{
    int32_t count = 0;
    uint32_t i;
    uint8_t crc8;
    uint16_t crc16;

    /* Check if the "best case" won't fit in the buffer which is
     * data_count + header + CRC16 = data_count + 6 bytes */
    if (CircularBuffer_SpaceLeft(Cbuff) < (data_count + 6))
        return HAL_FAILED;

    /* Add the header */
    /* Write the starting SYNC (without doubling it) */
    CircularBuffer_WriteSYNCNoIncrement(Cbuff, &count, &crc8, &crc16); 

    /* Add all of the header to the message */
    CircularBuffer_WriteNoIncrement(Cbuff, command,    &count, &crc8, &crc16); 
    CircularBuffer_WriteNoIncrement(Cbuff, data_count, &count, &crc8, &crc16); 
    CircularBuffer_WriteNoIncrement(Cbuff, crc8,       &count, NULL,  &crc16);

    /* Add the data to the message */
    for (i = 0; i < data_count; i++)
        CircularBuffer_WriteNoIncrement(Cbuff, data[i], &count, NULL, &crc16); 

    /* Add the CRC16 */
    CircularBuffer_WriteNoIncrement(Cbuff, (uint8_t)(crc16 >> 8), &count, NULL, NULL);
    CircularBuffer_WriteNoIncrement(Cbuff, (uint8_t)(crc16),      &count, NULL, NULL);

    /* Check if the message fit inside the buffer */
    return CircularBuffer_Increment(Cbuff, count);
}

/**
 * @brief                   Generates a message for the transmission of PI
 *                          controller data.
 * 
 * @param[in] command       Command to generate message for.
 * @param[in] pi_offset     Offset to the correct PI controller.
 * @param[in] limit_offset  Offset to the correct part of the Limits structure.
 * @param[in] limit_count   Number of bytes in Limit structure to read.
 * @param[out] Cbuff        Pointer to the circular buffer to put the data in.
 * @return                  HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                          if it did fit.
 */
bool GenerateGenericGetControllerData(KFly_Command_Type command, 
                                      const uint32_t pi_offset, 
                                      const uint32_t limit_offset, 
                                      const uint32_t limit_count, 
                                      Circular_Buffer_Type *Cbuff)
{
    (void)command;
    (void)pi_offset;
    (void)limit_offset;
    (void)limit_count;
    (void)Cbuff;

    return HAL_FAILED;

//    PI_Data_Type *PI_settings;
//    uint8_t *CL_settings;
//    uint8_t *data;
//    uint8_t crc8;
//    uint16_t crc16;
//    int32_t i, j;
//
//    /* The PI data is always 3 Controllers, with 3 gains plus
//       the limit structure */
//    const int32_t data_count = (3*3*4) + limit_count;
//    int32_t count = 0;
//
//    /* Check if the "best case" won't fit in the buffer */
//    if (CircularBuffer_SpaceLeft(Cbuff) < (data_count + 6))
//        return HAL_FAILED;
//
//    /* Add the header */
//    /* Write the starting SYNC (without doubling it) */
//    CircularBuffer_WriteSYNCNoIncrement(Cbuff, &count, &crc8, &crc16); 
//
//    /* Add all of the header to the message */
//    CircularBuffer_WriteNoIncrement(Cbuff, command,    &count, &crc8, &crc16); 
//    CircularBuffer_WriteNoIncrement(Cbuff, data_count, &count, &crc8, &crc16); 
//    CircularBuffer_WriteNoIncrement(Cbuff, crc8,       &count, NULL,  &crc16);
//
//    /* Cast the control data to an array of PI_Data_Type to access each
//       PI controller */
//    PI_settings = (PI_Data_Type *)ptrGetControlData();
//
//    /* Cast the settings into bytes for read out */
//    CL_settings = (uint8_t *)ptrGetControlLimits();
//
//    /* Get only the PI coefficients */
//    for (i = 0; i < 3; i++) 
//    {
//        data = (uint8_t *)&PI_settings[pi_offset + i];
//
//        for (j = 0; j < 12; j++)
//            CircularBuffer_WriteNoIncrement(Cbuff, data[j], &count, NULL, 
//                                                                    &crc16);
//    }
//
//    /* Get only the controller constraints */
//    for (i = 0; i < limit_count; i++) 
//        CircularBuffer_WriteNoIncrement(Cbuff, CL_settings[limit_offset + i],
//                                        &count, NULL,  &crc16);
//
//    /* Add the CRC16 */
//    CircularBuffer_WriteNoIncrement(Cbuff, (uint8_t)(crc16 >> 8), &count, NULL,
//                                                                          NULL);
//    CircularBuffer_WriteNoIncrement(Cbuff, (uint8_t)(crc16),      &count, NULL, 
//                                                                          NULL);
//
//    /* Check if the message fit inside the buffer */
//    return CircularBuffer_Increment(Cbuff, count);
}

/**
 * @brief               Generates an ACK.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateACK(Circular_Buffer_Type *Cbuff)
{
    return GenerateHeaderOnlyCommand(Cmd_ACK, Cbuff);   /* Return status */
}

/**
 * @brief               Generates a Ping.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GeneratePing(Circular_Buffer_Type *Cbuff)
{
    return GenerateHeaderOnlyCommand(Cmd_Ping, Cbuff);  /* Return status */
}

/**
 * @brief               Generates a Debug Message.
 * 
 * @param[in] data      Pointer to the data.
 * @param[in] size      Length of the data.
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateDebugMessage(uint8_t *data,
                          uint32_t size, 
                          Circular_Buffer_Type *Cbuff)
{
    if (size > 256)
        return HAL_FAILED;
    else
        return GenerateGenericCommand(Cmd_DebugMessage, data, size, Cbuff);
}

/**
 * @brief               Generates the message for the current running mode.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetRunningMode(Circular_Buffer_Type *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetRunningMode, (uint8_t *)"P", 1, Cbuff);
}

/**
 * @brief               Generates the message for the the ID of the system.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetDeviceInfo(Circular_Buffer_Type *Cbuff)
{
    uint8_t *device_id, *text_fw, *text_bl, *text_usr;
    uint32_t length_fw, length_bl, length_usr, data_count, i = 0;
    uint8_t crc8;
    uint16_t crc16;
    int32_t count = 0;

    /* The strings are at know location */
    device_id = ptrGetUniqueID();
    text_bl = ptrGetBootloaderVersion();
    text_fw = ptrGetFirmwareVersion();
    text_usr = ptrGetUserIDString();

    /* Find the length of the string */
    length_bl = myStrlen(text_bl, VERSION_MAX_SIZE);
    length_fw = myStrlen(text_fw, VERSION_MAX_SIZE);
    length_usr = myStrlen(text_usr, USER_ID_MAX_SIZE);

    /* The 3 comes from the 3 null bytes */
    data_count = UNIQUE_ID_SIZE + length_bl + length_fw + length_usr + 3;

    /* Check if the "best case" won't fit in the buffer */
    if (CircularBuffer_SpaceLeft(Cbuff) < (data_count + 6))
        return HAL_FAILED;

    /* Add the header */
    /* Write the starting SYNC (without doubling it) */
    CircularBuffer_WriteSYNCNoIncrement(                Cbuff, &count, &crc8, 
                                                                       &crc16); 

    /* Add all of the header to the message */
    CircularBuffer_WriteNoIncrement(Cbuff, Cmd_GetDeviceInfo, &count, &crc8, 
                                                                       &crc16); 
    CircularBuffer_WriteNoIncrement(Cbuff, data_count,        &count, &crc8,
                                                                       &crc16); 
    CircularBuffer_WriteNoIncrement(Cbuff, crc8,              &count, NULL,  
                                                                       &crc16);

    /* Get the Device ID */
    for (i = 0; i < UNIQUE_ID_SIZE; i++) 
        CircularBuffer_WriteNoIncrement(Cbuff, device_id[i], &count, NULL, 
                                                                       &crc16);

    /* Get the Bootloader Version string */
    for (i = 0; i < length_bl; i++) 
        CircularBuffer_WriteNoIncrement(Cbuff, text_bl[i], &count, NULL, 
                                                                       &crc16);

    CircularBuffer_WriteNoIncrement(Cbuff, 0x00, &count, NULL, 
                                                                       &crc16);

    /* Get the Firmware Version string */
    for (i = 0; i < length_fw; i++) 
        CircularBuffer_WriteNoIncrement(Cbuff, text_fw[i], &count, NULL, 
                                                                       &crc16);

    CircularBuffer_WriteNoIncrement(Cbuff, 0x00, &count, NULL, 
                                                                       &crc16);

    /* Get the User string */
    for (i = 0; i < length_usr; i++) 
        CircularBuffer_WriteNoIncrement(Cbuff, text_usr[i], &count, NULL, 
                                                                       &crc16);

    CircularBuffer_WriteNoIncrement(Cbuff, 0x00, &count, NULL, 
                                                                       &crc16);

    /* Add the CRC16 */
    CircularBuffer_WriteNoIncrement(Cbuff, (uint8_t)(crc16 >> 8), &count, NULL, 
                                                                          NULL);
    CircularBuffer_WriteNoIncrement(Cbuff, (uint8_t)(crc16),      &count, NULL, 
                                                                          NULL);

    /* Check if the message fit inside the buffer */
    return CircularBuffer_Increment(Cbuff, count);
}

/**
 * @brief               Generates the message for the rate controller data.
 * 
 * @param Cbuff[out]    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetRateControllerData(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericGetControllerData(Cmd_GetRateControllerData,
//                                            RATE_PI_OFFSET,
//                                            RATE_LIMIT_OFFSET,
//                                            RATE_LIMIT_COUNT,
//                                            Cbuff);
}

/**
 * @brief               Generates the message for the attitude controller data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * 
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetAttitudeControllerData(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericGetControllerData(Cmd_GetAttitudeControllerData,
//                                            ATTITUDE_PI_OFFSET,
//                                            ATTITUDE_LIMIT_OFFSET,
//                                            ATTITUDE_LIMIT_COUNT,
//                                            Cbuff);
}

/**
 * @brief               Generates the message for the velocity controller data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetVelocityControllerData(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericGetControllerData(Cmd_GetVelocityControllerData,
//                                            VELOCITY_PI_OFFSET,
//                                            VELOCITY_LIMIT_OFFSET,
//                                            VELOCITY_LIMIT_COUNT,
//                                            Cbuff);
}

/**
 * @brief               Generates the message for the position controller data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetPositionControllerData(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericGetControllerData(Cmd_GetPositionControllerData,
//                                            POSITION_PI_OFFSET,
//                                            POSITION_LIMIT_OFFSET,
//                                            POSITION_LIMIT_COUNT,
//                                            Cbuff);
}

/**
 * @brief               Generates the message for the output mixer.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS 
 *                      if it did fit.
 */
bool GenerateGetChannelMix(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetChannelMix, 
//                                  (uint8_t *)ptrGetOutputMixer(), 
//                                  OUTPUT_MIXER_SIZE, 
//                                  Cbuff);
}

/**
 * @brief               Generates the message for the RC Calibration.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetRCCalibration(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetRCCalibration, 
//                                  (uint8_t *)ptrGetRCInputSettings(), 
//                                  RC_INPUT_SETTINGS_SIZE, 
//                                  Cbuff);
}

/**
 * @brief               Generates the message to send current RC input.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetRCValues(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetRCCalibration, 
//                                  (uint8_t *)ptrGetRCRawInput(), 
//                                  RC_RAW_INPUT_SIZE, 
//                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the sensor data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetSensorData(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetSensorData, 
//                                  (uint8_t *)ptrGetSensorDataPointer(), 
//                                  (10*4), 
//                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the raw sensor data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetRawSensorData(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetRawSensorData, 
//                                  (uint8_t *)ptrGetRawSensorDataPointer(), 
//                                  (22), 
//                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the sensor
 *                      calibration data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetSensorCalibration(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetSensorCalibration, 
//                                  (uint8_t *)ptrGetSensorCalibration(), 
//                                  SENSOR_CALIBERATION_SIZE, 
//                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the
 *                      rate estimation data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetEstimationRate(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
}

/**
 * @brief               Generates the message for sending the attitude
 *                      estimation data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetEstimationAttitude(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
//    return GenerateGenericCommand(Cmd_GetEstimationAttitude,
//                                  (uint8_t *)ptrGetAttitudeEstimationStates(), 
//                                  ATTITUDE_ESTIMATION_STATES_SIZE, 
//                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the velocity 
 *                      estimation data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetEstimationVelocity(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
}

/**
 * @brief               Generates the message for sending the position 
 *                      estimation data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetEstimationPosition(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
}

/**
 * @brief               Generates the message for sending all the 
 *                      estimation data.
 * 
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
bool GenerateGetEstimationAllStates(Circular_Buffer_Type *Cbuff)
{
    (void)Cbuff;
    return HAL_FAILED;
}

/**
 * @brief                   Calculates the length of a string but with
 *                          maximum length termination.
 * 
 * @param[in] str           Pointer to the string.
 * @param[in] max_length    Maximum length/timeout.
 * @return                  Returns the length of the string.
 */
uint32_t myStrlen(const uint8_t *str, const uint32_t max_length)
{
    const uint8_t *s;
    s = str;

    while ((*s != '\0') && ((uint32_t)(s - str) < max_length))
        s++;

    return (s - str);
}
