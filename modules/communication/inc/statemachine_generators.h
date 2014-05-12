#ifndef __STATEMACHINE_GENERATORS_H
#define __STATEMACHINE_GENERATORS_H

/* Defines */

/* Typedefs */

/* Global variable defines */

/* Global function defines */
bool GenerateAUXMessage(KFly_Command_Type command, Port_Type port);
bool GenerateUSBMessage(KFly_Command_Type command);
bool GenerateHeaderOnlyCommand(KFly_Command_Type command, Circular_Buffer_Type *Cbuff);
bool GenerateGenericCommand(KFly_Command_Type command, uint8_t *data, const uint32_t data_count, Circular_Buffer_Type *Cbuff);
bool GenerateACK(Circular_Buffer_Type *Cbuff);
bool GeneratePing(Circular_Buffer_Type *Cbuff);
bool GenerateDebugMessage(uint8_t *data, uint32_t size, Circular_Buffer_Type *Cbuff);
bool GenerateGetRunningMode(Circular_Buffer_Type *Cbuff);
bool GenerateGetDeviceInfo(Circular_Buffer_Type *Cbuff);
bool GenerateGetRateControllerData(Circular_Buffer_Type *Cbuff);
bool GenerateGetAttitudeControllerData(Circular_Buffer_Type *Cbuff);
bool GenerateGetVelocityControllerData(Circular_Buffer_Type *Cbuff);
bool GenerateGetPositionControllerData(Circular_Buffer_Type *Cbuff);
bool GenerateGetChannelMix(Circular_Buffer_Type *Cbuff);
bool GenerateGetRCCalibration(Circular_Buffer_Type *Cbuff);
bool GenerateGetRCValues(Circular_Buffer_Type *Cbuff);
bool GenerateGetSensorData(Circular_Buffer_Type *Cbuff);
bool GenerateGetRawSensorData(Circular_Buffer_Type *Cbuff);
bool GenerateGetSensorCalibration(Circular_Buffer_Type *Cbuff);
bool GenerateGetSensorCalibration(Circular_Buffer_Type *Cbuff);
bool GenerateGetEstimationRate(Circular_Buffer_Type *Cbuff);
bool GenerateGetEstimationAttitude(Circular_Buffer_Type *Cbuff);
bool GenerateGetEstimationVelocity(Circular_Buffer_Type *Cbuff);
bool GenerateGetEstimationPosition(Circular_Buffer_Type *Cbuff);
bool GenerateGetEstimationAllStates(Circular_Buffer_Type *Cbuff);
uint32_t myStrlen(const uint8_t *str, const uint32_t max_length);

#endif
