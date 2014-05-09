#ifndef __SENSOR_CALIBRATION_H
#define __SENSOR_CALIBRATION_H

/* Defines */

/* Typedefs */

/* Global variable defines */
typedef struct
{
	float bias[3];
	float gain[3];
	mutex_t lock;
} Sensor_Calibration;

/* Global function defines */


#endif
