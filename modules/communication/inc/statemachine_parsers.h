#ifndef __STATEMACHINE_PARSERS_H
#define __STATEMACHINE_PARSERS_H

/* Defines */

/* Typedefs */

/* Global variable defines */

/* Global function defines */
Parser_Type GetParser(KFly_Command_Type command);
void ParsePing(Parser_Holder_Type *pHolder);
void ParseGetRunningMode(Parser_Holder_Type *pHolder);
void ParseGetDeviceInfo(Parser_Holder_Type *pHolder);
void ParseSetDeviceID(Parser_Holder_Type *pHolder);
void ParseSaveToFlash(Parser_Holder_Type *pHolder);
void ParseGetRateControllerData(Parser_Holder_Type *pHolder);
void ParseSetRateControllerData(Parser_Holder_Type *pHolder);
void ParseGetAttitudeControllerData(Parser_Holder_Type *pHolder);
void ParseSetAttitudeControllerData(Parser_Holder_Type *pHolder);
void ParseGetVelocityControllerData(Parser_Holder_Type *pHolder);
void ParseSetVelocityControllerData(Parser_Holder_Type *pHolder);
void ParseGetPositionControllerData(Parser_Holder_Type *pHolder);
void ParseSetPositionControllerData(Parser_Holder_Type *pHolder);
void ParseGetChannelMix(Parser_Holder_Type *pHolder);
void ParseSetChannelMix(Parser_Holder_Type *pHolder);
void ParseGetRCCalibration(Parser_Holder_Type *pHolder);
void ParseSetRCCalibration(Parser_Holder_Type *pHolder);
void ParseGetRCValues(Parser_Holder_Type *pHolder);
void ParseGetSensorData(Parser_Holder_Type *pHolder);
void ParseGetRawSensorData(Parser_Holder_Type *pHolder);
void ParseGetSensorCalibration(Parser_Holder_Type *pHolder);
void ParseSetSensorCalibration(Parser_Holder_Type *pHolder);
void ParseGetEstimationRate(Parser_Holder_Type *pHolder);
void ParseGetEstimationAttitude(Parser_Holder_Type *pHolder);
void ParseGetEstimationVelocity(Parser_Holder_Type *pHolder);
void ParseGetEstimationPosition(Parser_Holder_Type *pHolder);
void ParseGetEstimationAllStates(Parser_Holder_Type *pHolder);
void ParseResetEstimation(Parser_Holder_Type *pHolder);

#endif
