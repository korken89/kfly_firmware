/* *
 *
 * All the generators for the different serial messages.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "usb_access.h"
#include "version_information.h"
#include "serialmanager.h"
#include "slip.h"
#include "crc.h"
#include "pid.h"
#include "sensor_read.h"
#include "estimation.h"
#include "control.h"
#include "rc_input.h"
#include "kflypacket_parsers.h"
#include "kflypacket_generators.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

static bool GenerateACK(circular_buffer_t *Cbuff);
static bool GeneratePing(circular_buffer_t *Cbuff);
static bool GenerateGetRunningMode(circular_buffer_t *Cbuff);
static bool GenerateGetDeviceInfo(circular_buffer_t *Cbuff);
static bool GenerateGetControllerLimits(circular_buffer_t *Cbuff);
static bool GenerateGetArmSettings(circular_buffer_t *Cbuff);
static bool GenerateGetRateControllerData(circular_buffer_t *Cbuff);
static bool GenerateGetAttitudeControllerData(circular_buffer_t *Cbuff);
static bool GenerateGetVelocityControllerData(circular_buffer_t *Cbuff);
static bool GenerateGetPositionControllerData(circular_buffer_t *Cbuff);
static bool GenerateGetChannelMix(circular_buffer_t *Cbuff);
static bool GenerateGetRCCalibration(circular_buffer_t *Cbuff);
static bool GenerateGetRCValues(circular_buffer_t *Cbuff);
static bool GenerateGetSensorData(circular_buffer_t *Cbuff);
static bool GenerateGetRawSensorData(circular_buffer_t *Cbuff);
static bool GenerateGetSensorCalibration(circular_buffer_t *Cbuff);
static bool GenerateGetSensorCalibration(circular_buffer_t *Cbuff);
static bool GenerateGetEstimationRate(circular_buffer_t *Cbuff);
static bool GenerateGetEstimationAttitude(circular_buffer_t *Cbuff);
static bool GenerateGetEstimationVelocity(circular_buffer_t *Cbuff);
static bool GenerateGetEstimationPosition(circular_buffer_t *Cbuff);
static bool GenerateGetEstimationAllStates(circular_buffer_t *Cbuff);
static uint32_t myStrlen(const uint8_t *str, const uint32_t max_length);

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/**
 * The message generation lookup table
 */
static const kfly_generator_t generator_lookup[128] = {
    NULL,                             /* 0:   Cmd_None                        */
    GenerateACK,                      /* 1:   Cmd_ACK                         */
    GeneratePing,                     /* 2:   Cmd_Ping                        */
    NULL,                             /* 3:   Cmd_DebugMessage                */
    GenerateGetRunningMode,           /* 4:   Cmd_GetRunningMode              */
    NULL,                             /* 5:   Cmd_ManageSubscriptions         */
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
    GenerateGetControllerLimits,      /* 26:  Cmd_GetControllerLimits         */
    NULL,                             /* 27:  Cmd_SetControllerLimits         */
    GenerateGetArmSettings,           /* 28:  Cmd_GetArmSettings              */
    NULL,                             /* 29:  Cmd_SetArmSettings              */
    GenerateGetRateControllerData,    /* 30:  Cmd_GetRateControllerData       */
    NULL,                             /* 31:  Cmd_SetRateControllerData       */
    GenerateGetAttitudeControllerData,/* 32:  Cmd_GetAttitudeControllerData   */
    NULL,                             /* 33:  Cmd_SetAttitudeControllerData   */
    GenerateGetVelocityControllerData,/* 34:  Cmd_GetVelocityControllerData   */
    NULL,                             /* 35:  Cmd_SetVelocityControllerData   */
    GenerateGetPositionControllerData,/* 36:  Cmd_GetPositionControllerData   */
    NULL,                             /* 37:  Cmd_SetPositionControllerData   */
    NULL,                             /* 38:  RESERVED                        */
    GenerateGetChannelMix,            /* 39:  Cmd_GetChannelMix               */
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

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

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
static bool GenerateGenericGetControllerData(kfly_command_t command,
                                             const uint32_t pi_offset,
                                             const uint32_t limit_offset,
                                             const uint32_t limit_count,
                                             circular_buffer_t *Cbuff)
{
    (void) command;
    (void) pi_offset;
    (void) limit_offset;
    (void) limit_count;
    (void) Cbuff;

    return true;
//    PI_Data *PI_settings;
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
//    if (CircularBuffer_SpaceLeft(Cbuff) < (uint32_t)(data_count + 6))
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
//    /* Cast the control data to an array of PI_Data_ to access each
//       PI controller */
//    PI_settings = (PI_Data *)ptrGetControlData();
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
//    for (i = 0; i < (int32_t)limit_count; i++)
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
 * @brief               Generates a message with no data part.
 *
 * @param[in] command   Command to generate message for.
 * @param[in] Cbuff     Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateHeaderOnlyCommand(kfly_command_t command,
                                      circular_buffer_t *Cbuff)
{
    uint16_t crc16;
    uint8_t header[2] = {(uint8_t) command, 0};


    /* Calculate the CRC. */
    crc16 = CRC16(header, 2);

    /* Apply SLIP encoding on the fly and return the result. */
    return GenerateSLIP_HBT(header, 2, NULL, 0, (uint8_t *)&crc16, 2,
                            Cbuff);
}

/**
 * @brief              Generates a message with data and CRC16 part.
 *
 * @param[in] command  Command to generate message for.
 * @param[in] data     Pointer to where the data is located.
 * @param[in] size     Number of data bytes.
 * @param[out] Cbuff   Pointer to the circular buffer to put the data in.
 * @return             HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                     if it did fit.
 */
static bool GenerateGenericCommand(kfly_command_t command,
                                   uint8_t *data,
                                   const uint32_t size,
                                   circular_buffer_t *Cbuff)
{
    uint32_t i;
    uint16_t crc16;
    uint8_t header[2] = {(uint8_t) command, size};


    /* Calculate the CRC. */
    crc16 = CRC16_step((uint8_t)command, 0xffff);
    crc16 = CRC16_step(size, crc16);

    for (i = 0; i < size; i++)
        crc16 = CRC16_step(data[i], crc16);


    /* Apply SLIP encoding on the fly and return the result. */
    return GenerateSLIP_HBT(header, 2, data, size, (uint8_t *)&crc16, 2,
                            Cbuff);
}


/**
 * @brief               Generates an ACK.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateACK(circular_buffer_t *Cbuff)
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
static bool GeneratePing(circular_buffer_t *Cbuff)
{
    return GenerateHeaderOnlyCommand(Cmd_Ping, Cbuff);  /* Return status */
}

/**
 * @brief               Generates the message for the current running mode.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetRunningMode(circular_buffer_t *Cbuff)
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
static bool GenerateGetDeviceInfo(circular_buffer_t *Cbuff)
{
    uint8_t header[2];
    uint8_t *ptr_list[6];
    uint32_t len_list[6];
    uint32_t data_count, i;
    uint16_t crc16;

    /* The strings are at known locations. */
    ptr_list[0] = header;
    ptr_list[1] = (uint8_t *)ptrGetUniqueID();
    ptr_list[2] = (uint8_t *)ptrGetBootloaderVersion();
    ptr_list[3] = (uint8_t *)ptrGetFirmwareVersion();
    ptr_list[4] = ptrGetUserIDString();
    ptr_list[5] = (uint8_t *)&crc16;

    /* Find the length of the strings, + 1 for the null byte. */
    len_list[0] = 2;
    len_list[1] = UNIQUE_ID_SIZE;
    len_list[2] = myStrlen(ptr_list[2], VERSION_MAX_SIZE) + 1;
    len_list[3] = myStrlen(ptr_list[3], VERSION_MAX_SIZE) + 1;
    len_list[4] = myStrlen(ptr_list[4], USER_ID_MAX_SIZE) + 1;
    len_list[5] = 2;

    /* The 3 comes from the 3 null bytes. */
    data_count = UNIQUE_ID_SIZE + len_list[2] + len_list[3] + len_list[4] + 3;

    /* Fill header and build the CRC. */
    header[0] = Cmd_GetDeviceInfo;
    header[1] = (uint8_t) data_count;

    crc16 = CRC16_START_VALUE;

    for (i = 1; i < 5; i++)
        crc16 = CRC16_chunk(ptr_list[i], len_list[i], crc16);

    return GenerateSLIP_MultiChunk(ptr_list, len_list, 6, Cbuff);
}

/**
 * @brief               Generates the message for controller limits.
 *
 * @param Cbuff[out]    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetControllerLimits(circular_buffer_t *Cbuff)
{
    (void) Cbuff;

    return HAL_FAILED;
}

/**
 * @brief               Generates the message for controller arm settings.
 *
 * @param Cbuff[out]    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetArmSettings(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetArmSettings,
                                  (uint8_t *)ptrGetControlArmSettings(),
                                  CONTROL_ARM_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for the rate controller data.
 *
 * @param Cbuff[out]    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetRateControllerData(circular_buffer_t *Cbuff)
{
    return GenerateGenericGetControllerData(Cmd_GetRateControllerData,
                                            RATE_PI_OFFSET,
                                            RATE_LIMIT_OFFSET,
                                            RATE_LIMIT_COUNT,
                                            Cbuff);
}

/**
 * @brief               Generates the message for the attitude controller data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 *
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetAttitudeControllerData(circular_buffer_t *Cbuff)
{
    return GenerateGenericGetControllerData(Cmd_GetAttitudeControllerData,
                                            ATTITUDE_PI_OFFSET,
                                            ATTITUDE_LIMIT_OFFSET,
                                            ATTITUDE_LIMIT_COUNT,
                                            Cbuff);
}

/**
 * @brief               Generates the message for the velocity controller data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetVelocityControllerData(circular_buffer_t *Cbuff)
{
    return GenerateGenericGetControllerData(Cmd_GetVelocityControllerData,
                                            VELOCITY_PI_OFFSET,
                                            VELOCITY_LIMIT_OFFSET,
                                            VELOCITY_LIMIT_COUNT,
                                            Cbuff);
}

/**
 * @brief               Generates the message for the position controller data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetPositionControllerData(circular_buffer_t *Cbuff)
{
    return GenerateGenericGetControllerData(Cmd_GetPositionControllerData,
                                            POSITION_PI_OFFSET,
                                            POSITION_LIMIT_OFFSET,
                                            POSITION_LIMIT_COUNT,
                                            Cbuff);
}

/**
 * @brief               Generates the message for the output mixer.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetChannelMix(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetChannelMix,
                                  (uint8_t *)ptrGetOutputMixer(),
                                  OUTPUT_MIXER_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for the RC Calibration.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetRCCalibration(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetRCCalibration,
                                  (uint8_t *)ptrGetRCInputSettings(),
                                  RCINPUT_SETTINGS_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message to send current RC input.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetRCValues(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetRCValues,
                                  (uint8_t *)ptrGetRCInputData(),
                                  RCINPUT_DATA_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the sensor data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetSensorData(circular_buffer_t *Cbuff)
{
    /* Temporary IMU data holder */
    static imu_data_t imu_data;


    GetIMUData(&imu_data);
    return GenerateGenericCommand(Cmd_GetSensorData,
                                  (uint8_t *)&imu_data,
                                  SENSOR_IMU_DATA_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the raw sensor data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetRawSensorData(circular_buffer_t *Cbuff)
{
    /* Temporary IMU data holder */
    static imu_raw_data_t imu_rawdata;

    GetRawIMUData(&imu_rawdata);
    return GenerateGenericCommand(Cmd_GetRawSensorData,
                                  (uint8_t *)&imu_rawdata,
                                  SENSOR_IMU_RAW_DATA_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the sensor
 *                      calibration data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetSensorCalibration(circular_buffer_t *Cbuff)
{
    /* Temporary IMU data holder */
    static imu_calibration_t imu_calibration;

    GetIMUCalibration(&imu_calibration);

    return GenerateGenericCommand(Cmd_GetSensorCalibration,
                                  (uint8_t *)&imu_calibration,
                                  SENSOR_IMU_CALIBRATION_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the
 *                      rate estimation data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetEstimationRate(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetEstimationRate,
                                  ((uint8_t *)ptrGetAttitudeEstimationStates())
                                  + ESTIMATION_RATE_OFFSET,
                                  ESTIMATION_RATE_STATE_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the attitude
 *                      estimation data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetEstimationAttitude(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetEstimationAttitude,
                                  (uint8_t *)ptrGetAttitudeEstimationStates(),
                                  ESTIMATION_ATTITUDE_STATE_SIZE,
                                  Cbuff);
}

/**
 * @brief               Generates the message for sending the velocity
 *                      estimation data.
 *
 * @param[out] Cbuff    Pointer to the circular buffer to put the data in.
 * @return              HAL_FAILED if the message didn't fit or HAL_SUCCESS
 *                      if it did fit.
 */
static bool GenerateGetEstimationVelocity(circular_buffer_t *Cbuff)
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
static bool GenerateGetEstimationPosition(circular_buffer_t *Cbuff)
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
static bool GenerateGetEstimationAllStates(circular_buffer_t *Cbuff)
{
    return GenerateGenericCommand(Cmd_GetEstimationAllStates,
                                  (uint8_t *)ptrGetAttitudeEstimationStates(),
                                  ESTIMATION_STATES_SIZE,
                                  Cbuff);
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

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

 /**
  * @brief              Generate a message for the ports based on the
  *                     generators in the lookup table.
  *
  * @param[in] command  The command to generate a message for.
  * @param[in] port     Which port to send the data.
  * @return             HAL_FAILED if the message didn't fit or HAL_SUCCESS
  *                     if it did fit.
  */
bool GenerateMessage(kfly_command_t command, external_port_t port)
{
    bool status;
    circular_buffer_t *Cbuff = NULL;

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
  * @brief              Generate a message for the ports based on the
  *                     generators in the lookup table.
  *
  * @param[in] command  The command to generate a custom message for.
  * @param[in] data     Pointer to the data to be sent.
  * @param[in] size     Size of the data to be sent.
  * @param[in] port     Which port to send the data.
  * @return             HAL_FAILED if the message didn't fit or HAL_SUCCESS
  *                     if it did fit.
  */
bool GenerateCustomMessage(kfly_command_t command,
                           uint8_t *data,
                           uint16_t size,
                           external_port_t port)
{
    bool status;
    circular_buffer_t *Cbuff = NULL;

    Cbuff = SerialManager_GetCircularBufferFromPort(port);

    /* Check so the circular buffer address is valid */
    if (Cbuff == NULL)
        return HAL_FAILED;

    /* Claim the circular buffer for writing */
    CircularBuffer_Claim(Cbuff);
    {
        status = GenerateGenericCommand(command,
                                        data,
                                        size,
                                        Cbuff);
    }
    /* Release the circular buffer */
    CircularBuffer_Release(Cbuff);

    /* If it was successful then start the transmission */
    if (status == HAL_SUCCESS)
        SerialManager_StartTransmission(port);

    return status;
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
                          circular_buffer_t *Cbuff)
{
    if (size > 256)
        return HAL_FAILED;
    else
        return GenerateGenericCommand(Cmd_DebugMessage, data, size, Cbuff);
}
