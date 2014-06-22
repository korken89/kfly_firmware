#ifndef __SENSOR_READ_H
#define __SENSOR_READ_H

#include "mpu6050.h"
#include "hmc5983.h"
#include "sensor_calibration.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define ACCGYRO_DATA_AVAILABLE_EVENTMASK                EVENT_MASK(0)
#define MAG_DATA_AVAILABLE_EVENTMASK                    EVENT_MASK(1)
#define BARO_DATA_AVAILABLE_EVENTMASK                   EVENT_MASK(2)
#define SENSOR_IMU_DATA_SIZE                            (10*4)
#define SENSOR_IMU_RAW_DATA_SIZE                        (10*2 + 4)
#define SENSOR_IMU_CALIBRATION_SIZE                     (10*4 + 4)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Sensor read configuration structure.
 */
typedef struct
{
    /**
     * @brief   Pointer to the MPU6050 configuration.
     */
    const MPU6050_Configuration *mpu6050cfg;
    /**
     * @brief   Pointer to the HMC5983 configuration.
     */
    const HMC5983_Configuration *hmc5983cfg;
    /**
     * @brief   Pointer to MPU6050 calibration.
     */
    Sensor_Calibration *mpu6050cal;
    /**
     * @brief   Pointer to HMC5983 calibration.
     */
    Sensor_Calibration *hmc5983cal;
    /**
     * @brief   Pointer to calibration time stamp.
     */
    uint32_t *calibration_timestamp;
    /**
     * @brief   Pointer to the new data event source.
     */
    event_source_t *new_data_es;
} Sensor_Read_Configuration;

/**
 * @brief   Calibrated IMU data structure.
 */
typedef struct
{
    /**
     * @brief   Calibrated accelerometer data.
     */
    float accelerometer[3];
    /**
     * @brief   Calibrated gyroscope data.
     */
    float gyroscope[3];
    /**
     * @brief   Calibrated magnetometer data.
     */
    float magnetometer[3];
    /**
     * @brief   Temperature of the accelerometer and gyroscope.
     */
    float temperature;
} IMU_Data;

/**
 * @brief   Raw IMU data structure.
 */
typedef struct
{
    /**
     * @brief   Raw accelerometer data.
     */
    int16_t accelerometer[3];
    /**
     * @brief   Raw gyroscope data.
     */
    int16_t gyroscope[3];
    /**
     * @brief   Raw magnetometer data.
     */
    int16_t magnetometer[3];
    /**
     * @brief   Raw temperature of the accelerometer and gyroscope.
     */
    int16_t temperature;
    /**
     * @brief   Raw pressure of the barometer.
     */
    uint32_t pressure;
} IMU_RawData;

/**
 * @brief   IMU calibration data structure.
 */
typedef struct
{
    /**
     * @brief   Accelerometer bias.
     */
    float accelerometer_bias[3];
    /**
     * @brief   Accelerometer gain.
     */
    float accelerometer_gain[3];
    /**
     * @brief   Magnetometer bias.
     */
    float magnetometer_bias[3];
    /**
     * @brief   Magnetometer gain.
     */
    float magnetometer_gain[3];
    /**
     * @brief   Calibration time stamp.
     */
    uint32_t timestamp;
} IMU_Calibration;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
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
