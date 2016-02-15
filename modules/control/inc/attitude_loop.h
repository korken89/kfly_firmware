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
 * @param[in] limits                Control limits.
 * @param[in] dt                    Controller sampling time.
 */
static inline void vAttitudeControl(const quaternion_t *ref,
                                    const quaternion_t *attitude_m,
                                    vector3f_t *out,
                                    pi_data_t attitude_controller[3],
                                    const vector3f_t *limits,
                                    const float dt)
{
    vector3f_t err;

    /* TODO: Add the quaternion calculations. */
    /* Calculate the quaternion error */
    err.y = 0.0f;
    err.x = 0.0f;
    err.z = 0.0f;

    /* Update controllers, send bounded control signal to the next step */
    out->x =
        fPIUpdate_BC(&attitude_controller[0], err.x, limits->x, -limits->x, dt);
    out->y =
        fPIUpdate_BC(&attitude_controller[1], err.y, limits->y, -limits->y, dt);
    out->z =
        fPIUpdate_BC(&attitude_controller[2], err.z, limits->z, -limits->z, dt);
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
 * @param[in] limits                    Control limits.
 * @param[in] dt                        Controller sampling time.
 */
static inline void vAttitudeControlEuler(const vector3f_t *ref,
                                         const quaternion_t *attitude_m,
                                         vector3f_t *out,
                                         pi_data_t attitude_controller[3],
                                         const vector3f_t *limits,
                                         const float dt)
{
    vector3f_t err;

    /* TODO: Check the calculations */
    /* Calculate the attitude error */
    err.x = ref->x - atan2f(2.0f * (attitude_m->q0 * attitude_m->q1 +
                                    attitude_m->q2 * attitude_m->q3),
                            1.0f - 2.0f * (attitude_m->q1 * attitude_m->q1 +
                                           attitude_m->q2 * attitude_m->q2));

    err.y = ref->y - asinf(2.0f * (attitude_m->q0 * attitude_m->q2 -
                                   attitude_m->q1 * attitude_m->q3));

    /* Update controllers, send bounded control signal to the next step */
    out->x =
        fPIUpdate_BC(&attitude_controller[0], err.x, limits->x, -limits->x, dt);
    out->y =
        fPIUpdate_BC(&attitude_controller[1], err.y, limits->y, -limits->y, dt);
}
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/


#endif
