#ifndef __ATTITUDE_LOOP_H
#define __ATTITUDE_LOOP_H

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
 * @brief   Implements the quaternion attitude controller, inlined
 *          to minimize pointer passing.
 * @note    Assumes that yaw is controlled, else use the non yaw version.
 *
 * @param[in] ref                   Attitude reference.
 * @param[in] attitude_m            Attitude measurement.
 * @param[out] out                  Control output (rad/s).
 * @param[in] attitude_controller   Controller data structure.
 * @param[in] limits                Rate control limits.
 * @param[in] dt                    Controller sampling time.
 */
static inline void vAttitudeControl(const quaternion_t *ref,
                                    const quaternion_t *attitude_m,
                                    vector3f_t *out,
                                    pi_data_t attitude_controller[3],
                                    const vector3f_t *rate_limits,
                                    const float dt)
{
    /*
     * Based on the paper "Full quaternion based attitude control for a
     * quadrotor" by Emil Fresk, equations 19 and 20.
     */
    quaternion_t err;

    /* TODO: Check the calculations */
    /* Calculate the quaternion error */
    err = qmult(*ref, qconj(*attitude_m));

    /* Check the quaternion scalar to force minimum distance error. */
    if (err.w < 0.0f)
        err = qneg(err);

    /* Update controllers, send bounded control signal to the next step */
    out->x = fPIUpdate_BC(&attitude_controller[0],
                          err.x,
                          rate_limits->x,
                          -rate_limits->x,
                          dt);

    out->y = fPIUpdate_BC(&attitude_controller[1],
                          err.y,
                          rate_limits->y,
                          -rate_limits->y,
                          dt);

    out->z = fPIUpdate_BC(&attitude_controller[2],
                          err.z,
                          rate_limits->z,
                          -rate_limits->z,
                          dt);
}

/**
 * @brief   Implements an Euler angle attitude controller, inlined
 *          to minimize pointer passing.
 * @note    Assumes that yaw is not controlled, else use the yaw version.
 *
 * @param[in] ref                       Attitude reference around roll (x) and
 *                                      pitch (y) in radians, z is unused.
 * @param[in] attitude_m                Attitude measurement.
 * @param[out] out                      Control output (rad/s).
 * @param[in/out] attitude_controller   Controller data structure.
 * @param[in] limits                    Rate control limits.
 * @param[in] dt                        Controller sampling time.
 */
static inline void vAttitudeControlEuler(const vector3f_t *ref,
                                         const quaternion_t *attitude_m,
                                         vector3f_t *out,
                                         pi_data_t attitude_controller[3],
                                         const vector3f_t *rate_limits,
                                         const float dt)
{
    vector3f_t err;

    /* TODO: Check the calculations */
    /* Calculate the attitude error */
    err.x = ref->x - atan2f(2.0f * (attitude_m->w * attitude_m->x +
                                    attitude_m->y * attitude_m->z),
                            1.0f - 2.0f * (attitude_m->x * attitude_m->x +
                                           attitude_m->y * attitude_m->y));

    err.y = ref->y - asinf(2.0f * (attitude_m->w * attitude_m->y -
                                   attitude_m->x * attitude_m->z));

    /* Update controllers, send bounded control signal to the next step */
    out->x = fPIUpdate_BC(&attitude_controller[0],
                          err.x,
                          rate_limits->x,
                          -rate_limits->x,
                          dt);

    out->y = fPIUpdate_BC(&attitude_controller[1],
                          err.y,
                          rate_limits->y,
                          -rate_limits->y,
                          dt);

    out->z = bound(rate_limits->z, -rate_limits->z, ref->z);
}
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/


#endif
