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
                                pi_data_t rate_controller[3],
                                const float dt)
{
    vector3f_t error;

    /* Calculate the errors */
    error = vector_sub(*ref, *omega_m);

    /* Update the PI controllers */
    out->x = fPIUpdate_BC(&rate_controller[0], error.x, 1.0f, -1.0f, dt);
    out->y = fPIUpdate_BC(&rate_controller[1], error.y, 1.0f, -1.0f, dt);
    out->z = fPIUpdate_BC(&rate_controller[2], error.z, 1.0f, -1.0f, dt);
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/


#endif
