/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "control_reference.h"
#include "arming.h"
#include "rc_input.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define DEG2RAD                                 (0.0174532925199433f)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief       Converts RC inputs to control action depending on the current
 *              flight mode.
 * @note        Only for usage in manual control mode.
 *
 * @param[out] ref          Reference output.
 * @param[in] rate_lim      Rate limits around roll (x), pitch (y) and yaw (z).
 * @param[in] attitude_lim  Attitude limits around roll (x) and pitch (y).
 */
void RCInputsToControlAction(control_reference_t *ref,
                             const vector3f_t *rate_lim,
                             const vector3f_t *attitude_lim)
{
    if (ref->mode == FLIGHTMODE_RATE)
    {
        ref->actuator_desired.throttle = RCInputGetInputLevel(ROLE_THROTTLE);

        ref->rate_reference.x =
            rate_lim->x * DEG2RAD * RCInputGetInputLevel(ROLE_PITCH);
        ref->rate_reference.y =
            rate_lim->y * DEG2RAD * RCInputGetInputLevel(ROLE_ROLL);
        ref->rate_reference.z =
            rate_lim->z * DEG2RAD * RCInputGetInputLevel(ROLE_YAW);
    }
    else if (ref->mode == FLIGHTMODE_ATTITUDE_EULER)
    {
        ref->actuator_desired.throttle = RCInputGetInputLevel(ROLE_THROTTLE);

        ref->attitude_reference_euler.x =
            attitude_lim->x * DEG2RAD * RCInputGetInputLevel(ROLE_ROLL);
        ref->attitude_reference_euler.y =
            attitude_lim->y * DEG2RAD * RCInputGetInputLevel(ROLE_PITCH);
        ref->rate_reference.z =
            attitude_lim->z * DEG2RAD * RCInputGetInputLevel(ROLE_YAW);
    }
    else
    {
        /* The other modes (quaternion etc) are not supported here yet. */
        ref->actuator_desired.throttle = 0.0f;
    }
}

