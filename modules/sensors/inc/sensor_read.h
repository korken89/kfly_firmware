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

/* Global variable defines */

/* Global function defines */
msg_t SensorReadInit(void);
void MPU6050cb(EXTDriver *extp, expchannel_t channel);
void HMC5983cb(EXTDriver *extp, expchannel_t channel);

#endif
