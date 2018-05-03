#ifndef __RATE_LOOP_H
#define __RATE_LOOP_H

#include "quaternion.h"
#include "trigonometry.h"
#include "pid.h"

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

/**
 * @brief   Implements the rate PI controller.
 *
 * @param[in] ref                   Rate reference around roll (x), pitch (y)
 *                                  and yaw (z) in rad/s.
 * @param[in] rate_m                Rate measurement.
 * @param[out] out                  Control output in -100 to 100 % torque.
 * @param[in/out] rate_controller   Controller data structure.
 * @param[in] dt                    Controller sampling time.
 */
static inline void vRateControl(const vector3f_t *ref,
                                const vector3f_t *omega_m,
                                vector3f_t *out,
                                pid_data_t rate_controller[3],
                                biquad_df2t_t dterm_filter[3],
                                const float dt)
{
    vector3f_t error;

    /* Calculate the errors */
    error = vector_sub(*ref, *omega_m);

    /* Update the PI controllers */
    out->x = fPIDUpdate_BC(&rate_controller[0], &dterm_filter[0], error.x, 0.4f,
                           -0.4f, dt);
    out->y = fPIDUpdate_BC(&rate_controller[1], &dterm_filter[1], error.y, 0.4f,
                           -0.4f, dt);
    out->z = fPIDUpdate_BC(&rate_controller[2], &dterm_filter[2], error.z, 0.4f,
                           -0.4f, dt);
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/


#endif
