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
} sensor_calibration_t;

/* Global function defines */


#endif
