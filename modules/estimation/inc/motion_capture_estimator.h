#ifndef __MOTION_CAPTURE_ESTIMATOR_H
#define __MOTION_CAPTURE_ESTIMATOR_H

#include <math.h>
#include "quaternion.h"
#include "linear_algebra.h"
#include "trigonometry.h"
#include "sensor_read.h"
#include "attitude_ekf.h"
#include "motion_capture.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vInitializeMotionCaptureEstimator(attitude_states_t *states);
void vInnovateMotionCaptureEstimator(attitude_states_t *states,
                                     imu_data_t *imu_data,
                                     const float dt,
                                     const float wb_gain,
                                     const float gyro_lpf);

#endif /* __VICON_ESTIMATOR_H */
