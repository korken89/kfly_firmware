#ifndef __VICON_ESTIMATOR_H
#define __VICON_ESTIMATOR_H

#include <math.h>
#include "quaternion.h"
#include "linear_algebra.h"
#include "trigonometry.h"
#include "attitude_ekf.h"
#include "vicon.h"

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
void vInitializeViconEstimator(attitude_states_t *states);
void vInnovateViconEstimator(attitude_states_t *states,
                             vector3f_t gyro_rate,
                             const float dt,
                             const float gyro_lpf);

#endif /* __VICON_ESTIMATOR_H */
