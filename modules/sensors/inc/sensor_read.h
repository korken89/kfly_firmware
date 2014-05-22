#ifndef __SENSOR_READ_H
#define __SENSOR_READ_H

/* Defines */
#define MS5611_DATA_AVAILABLE_EVENTMASK     0x04

/* Typedefs */
typedef struct
{
	const MPU6050_Configuration *mpu6050cfg;
	const HMC5983_Configuration *hmc5983cfg;
	Sensor_Calibration *mpu6050cal;
    Sensor_Calibration *hmc5983cal;
} Sensor_Read_Configuration;

typedef struct
{
	float accelerometer[3];
	float gyroscope[3];
	float magnetometer[3];
	float temperature;
} IMU_Data;

/* Global variable defines */

/* Global function defines */
msg_t SensorReadInit(void);
void SetSensorCalibration(Sensor_Calibration *cal, float bias[3], float gain[3]);
void MPU6050cb(EXTDriver *extp, expchannel_t channel);
void HMC5983cb(EXTDriver *extp, expchannel_t channel);
event_source_t *ptrGetAccelerometerAndGyroscopeEventSource(eventmask_t *mask);
event_source_t *ptrGetMagnetometerEventSource(eventmask_t *mask);
int16_t *ptrGetRawAccelerometerData(void);
float *ptrGetAccelerometerData(void);
int16_t *ptrGetRawGyroscopeData(void);
float *ptrGetGyroscopeData(void);
int16_t GetRawGyroscopeTemperature(void);
float GetGyroscopeTemperature(void);
int16_t *ptrGetRawMagnetometerData(void);
float *ptrGetMagnetometerData(void);
void GetIMUData(IMU_Data *data);
void LockSensorStructuresForRead(void);
void UnlockSensorStructuresForRead(void);
Sensor_Calibration *ptrGetAccelerometerCalibration(void);
Sensor_Calibration *ptrGetMagnetometerCalibration(void);

#endif
