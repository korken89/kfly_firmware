#ifndef __SENSOR_READ_H
#define __SENSOR_READ_H

#include "mpu6050.h"
#include "hmc5983.h"
#include "sensor_calibration.h"

/* Defines */
#define ACCGYRO_DATA_AVAILABLE_EVENTMASK    EVENT_MASK(0)
#define MAG_DATA_AVAILABLE_EVENTMASK        EVENT_MASK(1)
#define BARO_DATA_AVAILABLE_EVENTMASK       EVENT_MASK(2)
#define SENSOR_IMU_DATA_SIZE                (10*4)
#define SENSOR_IMU_RAW_DATA_SIZE            (10*2)
#define SENSOR_IMU_CALIBRATION_SIZE         (10*4 + 4)

/* Typedefs */
typedef struct
{
    const MPU6050_Configuration *mpu6050cfg;
    const HMC5983_Configuration *hmc5983cfg;
    Sensor_Calibration *mpu6050cal;
    Sensor_Calibration *hmc5983cal;
    uint32_t *calibration_timestamp;
    event_source_t *new_data_es;
} Sensor_Read_Configuration;

typedef struct
{
    float accelerometer[3];
    float gyroscope[3];
    float magnetometer[3];
    float temperature;
} IMU_Data;

typedef struct
{
    int16_t accelerometer[3];
    int16_t gyroscope[3];
    int16_t magnetometer[3];
    int16_t temperature;
} IMU_RawData;

typedef struct
{
    float accelerometer_bias[3];
    float accelerometer_gain[3];
    float magnetometer_bias[3];
    float magnetometer_gain[3];
    uint32_t timestamp;
} IMU_Calibration;

/* Global variable defines */

/* Global function defines */
msg_t SensorReadInit(void);
void MPU6050cb(EXTDriver *extp, expchannel_t channel);
void HMC5983cb(EXTDriver *extp, expchannel_t channel);
event_source_t *ptrGetNewDataEventSource(void);
int16_t *ptrGetRawAccelerometerData(void);
float *ptrGetAccelerometerData(void);
int16_t *ptrGetRawGyroscopeData(void);
float *ptrGetGyroscopeData(void);
int16_t GetRawGyroscopeTemperature(void);
float GetGyroscopeTemperature(void);
int16_t *ptrGetRawMagnetometerData(void);
float *ptrGetMagnetometerData(void);
void GetIMUData(IMU_Data *data);
void GetRawIMUData(IMU_RawData *data);
void GetIMUCalibration(IMU_Calibration *cal);
void SetIMUCalibration(IMU_Calibration *cal);
void LockSensorStructures(void);
void UnlockSensorStructures(void);
void LockSensorCalibration(void);
void UnlockSensorCalibration(void);
Sensor_Calibration *ptrGetAccelerometerCalibration(void);
Sensor_Calibration *ptrGetMagnetometerCalibration(void);

#endif
