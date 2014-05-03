#ifndef __SENSOR_READ_H
#define __SENSOR_READ_H

/* Defines */
#define MS5611_DATA_AVAILABLE_MASK		0x04

/* Typedefs */

/* Global variable defines */

/* Global function defines */
msg_t SensorReadInit(const MPU6050_Configuration *mpu6050cfg,
					 const HMC5983_Configuration *mhc5983cfg);

void MPU6050cb(EXTDriver *extp, expchannel_t channel);
void HMC5983cb(EXTDriver *extp, expchannel_t channel);
int16_t twoscomplement2signed(uint8_t msb, uint8_t lsb);

#endif
