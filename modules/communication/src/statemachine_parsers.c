/* *
 *
 * All the parsers for the different packages.
 * To expand the functionality of the serial communication is just to add
 * functions here and add them to the parser_lookup table.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "flash_save.h"
#include "version_information.h"
#include "statemachine_generators.h"
#include "serialmanager.h"
#include "crc.h"
#include "pid.h"
#include "rc_input.h"
#include "sensor_read.h"
#include "estimation.h"
#include "control.h"
#include "vicon.h"
#include "statemachine_parsers.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

static void ParsePing(parser_holder_t *pHolder);
static void ParseGetRunningMode(parser_holder_t *pHolder);
static void ParseManageSubscriptions(parser_holder_t *pHolder);
static void ParseGetDeviceInfo(parser_holder_t *pHolder);
static void ParseSetDeviceID(parser_holder_t *pHolder);
static void ParseSaveToFlash(parser_holder_t *pHolder);
static void ParseSetArmSettings(parser_holder_t *pHolder);
static void ParseGetArmSettings(parser_holder_t *pHolder);
static void ParseGetRateControllerData(parser_holder_t *pHolder);
static void ParseSetRateControllerData(parser_holder_t *pHolder);
static void ParseGetAttitudeControllerData(parser_holder_t *pHolder);
static void ParseSetAttitudeControllerData(parser_holder_t *pHolder);
static void ParseGetVelocityControllerData(parser_holder_t *pHolder);
static void ParseSetVelocityControllerData(parser_holder_t *pHolder);
static void ParseGetPositionControllerData(parser_holder_t *pHolder);
static void ParseSetPositionControllerData(parser_holder_t *pHolder);
static void ParseGetChannelMix(parser_holder_t *pHolder);
static void ParseSetChannelMix(parser_holder_t *pHolder);
static void ParseGetRCCalibration(parser_holder_t *pHolder);
static void ParseSetRCCalibration(parser_holder_t *pHolder);
static void ParseGetRCValues(parser_holder_t *pHolder);
static void ParseGetSensorData(parser_holder_t *pHolder);
static void ParseGetRawSensorData(parser_holder_t *pHolder);
static void ParseGetSensorCalibration(parser_holder_t *pHolder);
static void ParseSetSensorCalibration(parser_holder_t *pHolder);
static void ParseGetEstimationRate(parser_holder_t *pHolder);
static void ParseGetEstimationAttitude(parser_holder_t *pHolder);
static void ParseGetEstimationVelocity(parser_holder_t *pHolder);
static void ParseGetEstimationPosition(parser_holder_t *pHolder);
static void ParseGetEstimationAllStates(parser_holder_t *pHolder);
static void ParseResetEstimation(parser_holder_t *pHolder);
static void ParseViconMeasurement(parser_holder_t *pHolder);

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief Lookup table for all the serial parsers.
 */
static const parser_t parser_lookup[128] = {
    NULL,                             /* 0:   Cmd_None                        */
    NULL,                             /* 1:   Cmd_ACK                         */
    ParsePing,                        /* 2:   Cmd_Ping                        */
    NULL,                             /* 3:   Cmd_DebugMessage                */
    ParseGetRunningMode,              /* 4:   Cmd_GetRunningMode              */
    ParseManageSubscriptions,         /* 5:   Cmd_ManageSubscriptions         */
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
    ParseGetDeviceInfo,               /* 17:  Cmd_GetBootloaderVersion        */
    ParseSetDeviceID,                 /* 18:  Cmd_SetDeviceID                 */
    ParseSaveToFlash,                 /* 19:  Cmd_SaveToFlash                 */
    NULL,                             /* 20:                                  */
    NULL,                             /* 21:                                  */
    NULL,                             /* 22:                                  */
    NULL,                             /* 23:                                  */
    NULL,                             /* 24:                                  */
    NULL,                             /* 25:                                  */
    NULL,                             /* 26:                                  */
    NULL,                             /* 27:                                  */
    ParseGetArmSettings,              /* 28:  Cmd_GetArmSettings              */
    ParseSetArmSettings,              /* 29:  Cmd_SetArmSettings              */
    ParseGetRateControllerData,       /* 30:  Cmd_GetRateControllerData       */
    ParseSetRateControllerData,       /* 31:  Cmd_SetRateControllerData       */
    ParseGetAttitudeControllerData,   /* 32:  Cmd_GetAttitudeControllerData   */
    ParseSetAttitudeControllerData,   /* 33:  Cmd_SetAttitudeControllerData   */
    ParseGetVelocityControllerData,   /* 34:  Cmd_GetVelocityControllerData   */
    ParseSetVelocityControllerData,   /* 35:  Cmd_SetVelocityControllerData   */
    ParseGetPositionControllerData,   /* 36:  Cmd_GetPositionControllerData   */
    ParseSetPositionControllerData,   /* 37:  Cmd_SetPositionControllerData   */
    NULL,                             /* 38:  RESERVED                        */
    ParseGetChannelMix,               /* 39:  Cmd_GetChannelMix               */
    ParseSetChannelMix,               /* 40:  Cmd_SetChannelMix               */
    ParseGetRCCalibration,            /* 41:  Cmd_GetRCCalibration            */
    ParseSetRCCalibration,            /* 42:  Cmd_SetRCCalibration            */
    ParseGetRCValues,                 /* 43:  Cmd_GetRCValues                 */
    ParseGetSensorData,               /* 44:  Cmd_GetSensorData               */
    ParseGetRawSensorData,            /* 45:  Cmd_GetRawSensorData            */
    ParseGetSensorCalibration,        /* 46:  Cmd_GetSensorCalibration        */
    ParseSetSensorCalibration,        /* 47:  Cmd_SetSensorCalibration        */
    ParseGetEstimationRate,           /* 48:  Cmd_GetEstimationRate           */
    ParseGetEstimationAttitude,       /* 49:  Cmd_GetEstimationAttitude       */
    ParseGetEstimationVelocity,       /* 50:  Cmd_GetEstimationVelocity       */
    ParseGetEstimationPosition,       /* 51:  Cmd_GetEstimationPosition       */
    ParseGetEstimationAllStates,      /* 52:  Cmd_GetEstimationAllStates      */
    ParseResetEstimation,             /* 53:  Cmd_ResetEstimation             */
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
    ParseViconMeasurement             /* 127: Cmd_ViconMeasurement            */
};

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief                   A generic function to save data. Locks the RTOS
 *                          while saving.
 * 
 * @param[in] save_location Pointer to the location where the data shall be
 *                          saved.
 * @param[in] buffer        Pointer to the buffer where to save from.
 * @param[in] data_length   Number of bytes to save
 */
static void GenericSaveData(uint8_t *save_location,
                            uint8_t *buffer,
                            uint32_t data_length)
{
    uint32_t i;

    /* Lock while saving the data */
    osalSysLock();

    /* Save the string */
    for (i = 0; i < data_length; i++) 
            save_location[i] = buffer[i];

    osalSysUnlock();
}

/**
 * @brief                   A generic function to save controller data and
 *                          control limits.
 * 
 * @param[in] pi_offset     The offset to the correct set of PI controllers.
 * @param[in] limit_offset  The offset to the correct part of the limits
 *                          structure.
 * @param[in] limit_count   Number of bytes to write to the limits structure.
 * @param[in] data          Input data so save.
 */
static void ParseGenericSetControllerData(const uint32_t pi_offset,
                                          const uint32_t limit_offset, 
                                          const uint32_t limit_count, 
                                          uint8_t *data)
{
    PI_Data *PI_settings;
    uint8_t *save_location;
    uint32_t i, j;

    /* Lock while saving the data */
    osalSysLock();

    /* Cast the control data to an array of PI_Data
    to access each PI controller */
    PI_settings = (PI_Data *)ptrGetControlData();

    /* Write only the PI coefficients */
    for (i = 0; i < 3; i++) 
    {
        save_location = (uint8_t *)&PI_settings[pi_offset + i];

        for (j = 0; j < 12; j++)
            save_location[j] = data[(i*12) + j];
    }

    /* Cast the settings into bytes for saving */
    save_location = (uint8_t *)ptrGetControlLimits();

    /* Write only the controller constraints */
    for (i = 0; i < limit_count; i++) 
        save_location[limit_offset + i] = data[(3*3*4) + i];

    osalSysUnlock();
}

/**
 * @brief               Parses a Ping command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission. 
 */
static void ParsePing(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_Ping, pHolder->Port);
}

/**
 * @brief               Parses a GetRunningMode command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetRunningMode(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetRunningMode, pHolder->Port);
}

/**
 * @brief               Parses a ManageSubscriptions command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseManageSubscriptions(parser_holder_t *pHolder)
{
    /* Parsing structure for the data */
    subscription_parser_t *p;

    /* Check so the length of the message is correct */
    if (pHolder->data_length == sizeof(subscription_parser_t))
    {
        /* Cast the message to the parser structure */
        p = (subscription_parser_t *)pHolder->buffer;

        /* Check for valid port */
        if ((isPort(p->port) == true) || (p->port == 0xff))
        {
            if (p->on_off == 0)
            {
                /* Unsubscribe from command */
                if (p->port == 0xff) /* Port is the one the command came on */
                    UnsubscribeFromCommand(p->command, pHolder->Port);
                else /* Port is is specified in the message */
                    UnsubscribeFromCommand(p->command, p->port);
            }
            else
            {
                /* Subscribe to command */
                if (p->port == 0xff) /* Port is the one the command came on */
                    SubscribeToCommand(p->command,
                                       pHolder->Port,
                                       p->delta_time);
                else /* Port is is specified in the message */
                    SubscribeToCommand(p->command,
                                       p->port,
                                       p->delta_time);
            }
        }
    }
}

/**
 * @brief               Parses a GetDeviceInfo command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetDeviceInfo(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetDeviceInfo, pHolder->Port);
}

/**
 * @brief               Parses a SetDeviceID command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetDeviceID(parser_holder_t *pHolder)
{
    uint8_t *save_location;

    /* Check so the wasn't to big a string received */
    if (pHolder->buffer_count > USER_ID_MAX_SIZE)
        return;

    save_location = ptrGetUserIDString();

    /* Save the string */
    GenericSaveData(save_location,
                    pHolder->buffer,
                    pHolder->data_length);

    /* Add a trailing zero to end the string */
    save_location[pHolder->data_length] = 0x00;
}

/**
 * @brief               Parses a SaveToFlash command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSaveToFlash(parser_holder_t *pHolder)
{
    (void)pHolder;

    /* Broadcast the Save to Flash event */
    vBroadcastFlashSaveEvent();
}

/**
 * @brief               Parses a SetArmSettings command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetArmSettings(parser_holder_t *pHolder)
{
    if (pHolder->buffer_count == CONTROL_ARM_SIZE)
    {
        /* Save the data */
        GenericSaveData((uint8_t *)ptrGetControlArmSettings(),
                        pHolder->buffer,
                        CONTROL_ARM_SIZE);
    }
}

/**
 * @brief               Parses a GetArmSettings command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetArmSettings(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetArmSettings, pHolder->Port);
}

/**
 * @brief               Parses a GetRateControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetRateControllerData(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetRateControllerData, pHolder->Port);
}

/**
 * @brief               Parses a SetRateControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetRateControllerData(parser_holder_t *pHolder)
{
    if (pHolder->buffer_count == (36 + RATE_LIMIT_COUNT))
        ParseGenericSetControllerData(RATE_PI_OFFSET,
                                      RATE_LIMIT_OFFSET, 
                                      RATE_LIMIT_COUNT, 
                                      pHolder->buffer);
}

/**
 * @brief               Parses a GetAttitudeControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetAttitudeControllerData(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetAttitudeControllerData, pHolder->Port);
}

/**
 * @brief               Parses a SetAttitudeControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetAttitudeControllerData(parser_holder_t *pHolder)
{
    if (pHolder->buffer_count == (36 + ATTITUDE_LIMIT_COUNT))
        ParseGenericSetControllerData(ATTITUDE_PI_OFFSET,
                                      ATTITUDE_LIMIT_OFFSET,
                                      ATTITUDE_LIMIT_COUNT,
                                      pHolder->buffer);
}

/**
 * @brief               Parses a GetVelocityControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetVelocityControllerData(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetVelocityControllerData, pHolder->Port);
}

/**
 * @brief               Parses a SetVelocityControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetVelocityControllerData(parser_holder_t *pHolder)
{
    (void)pHolder;
    if (pHolder->buffer_count == (36 + VELOCITY_LIMIT_COUNT))
        ParseGenericSetControllerData(VELOCITY_PI_OFFSET,
                                      VELOCITY_LIMIT_OFFSET,
                                      VELOCITY_LIMIT_COUNT,
                                      pHolder->buffer);    
}

/**
 * @brief               Parses a GetPositionControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetPositionControllerData(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetPositionControllerData, pHolder->Port);
}

/**
 * @brief               Parses a SetPositionControllerData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetPositionControllerData(parser_holder_t *pHolder)
{
    if (pHolder->buffer_count == (36 + POSITION_LIMIT_COUNT))
        ParseGenericSetControllerData(POSITION_PI_OFFSET, 
                                      POSITION_LIMIT_OFFSET, 
                                      POSITION_LIMIT_COUNT, 
                                      pHolder->buffer);
}

/**
 * @brief               Parses a GetChannelMix command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetChannelMix(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetChannelMix, pHolder->Port);
}

/**
 * @brief               Parses a SetChannelMix command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetChannelMix(parser_holder_t *pHolder)
{
    if (pHolder->buffer_count == OUTPUT_MIXER_SIZE)
    {
        /* Save the data */
        GenericSaveData((uint8_t *)ptrGetOutputMixer(),
                        pHolder->buffer,
                        OUTPUT_MIXER_SIZE);
    }
}

/**
 * @brief               Parses a GetRCCalibration command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetRCCalibration(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetRCCalibration, pHolder->Port);
}

/**
 * @brief               Parses a SetRCCalibration command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetRCCalibration(parser_holder_t *pHolder)
{
    if (pHolder->buffer_count == RCINPUT_SETTINGS_SIZE)
    {
        /* Save the data */
        GenericSaveData((uint8_t *)ptrGetRCInputSettings(),
                        pHolder->buffer,
                        RCINPUT_SETTINGS_SIZE);
    }

    /* Reinitialize the RC Input module */
    RCInputInitialization();
}

/**
 * @brief               Parses a GetRCValues command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetRCValues(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetRCValues, pHolder->Port);
}

/**
 * @brief               Parses a GetSensorData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetSensorData(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetSensorData, pHolder->Port);
}

/**
 * @brief               Parses a GetRawSensorData command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetRawSensorData(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetRawSensorData, pHolder->Port);
}

/**
 * @brief               Parses a GetSensorCalibration command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetSensorCalibration(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetSensorCalibration, pHolder->Port);
}

/**
 * @brief               Parses a SetSensorCalibration command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseSetSensorCalibration(parser_holder_t *pHolder)
{
    /* Temporary holder for IMU calibration data */
    static IMU_Calibration imu_calibration;

    /* Save the data into the temporary calibration structure */
    if (pHolder->buffer_count == SENSOR_IMU_CALIBRATION_SIZE)
    {
        /* Save the data */
        GenericSaveData((uint8_t *)&imu_calibration,
                        pHolder->buffer,
                        SENSOR_IMU_CALIBRATION_SIZE);

        /* Move the calibration data into the sensor structures */
        SetIMUCalibration(&imu_calibration);

        /* Reset estimation */
        ResetEstimation();
    }
}

/**
 * @brief               Parses a GetEstimationRate command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetEstimationRate(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetEstimationRate, pHolder->Port);
}

/**
 * @brief               Parses a GetEstimationAttitude command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetEstimationAttitude(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetEstimationAttitude, pHolder->Port);
}

/**
 * @brief               Parses a GetEstimationVeocity command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetEstimationVelocity(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetEstimationVelocity, pHolder->Port);
}

/**
 * @brief               Parses a GetEstimationPosition command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetEstimationPosition(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetEstimationPosition, pHolder->Port);
}

/**
 * @brief               Parses a GetEstimationAllStates command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseGetEstimationAllStates(parser_holder_t *pHolder)
{
    GenerateMessage(Cmd_GetEstimationAllStates, pHolder->Port);
}

/**
 * @brief               Parses a ResetEstimation command.
 * 
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseResetEstimation(parser_holder_t *pHolder)
{
    (void)pHolder;
}

/**
 * @brief               Parses a ViconMeasurement command.
 *
 * @param[in] pHolder   Message holder containing information
 *                      about the transmission.
 */
static void ParseViconMeasurement(parser_holder_t *pHolder)
{
    (void)pHolder;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Return the parser associated to the command.
 * 
 * @param[in] command   Command to get the parser for.
 * @return              Pointer to the associated parser.
 */
parser_t GetParser(KFly_Command command)
{
    return parser_lookup[command];
}
