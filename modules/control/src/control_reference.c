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

control_reference_t control_reference;

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
 * @brief           Converts RC inputs to control action depending on
 *                  the current flight mode.
 */
void UpdateRCInputsToControlAction(control_reference_t *ref)
{
    float throttle = 0.0f;

    if (ref->mode == FLIGHTMODE_RATE)
    {
        control_reference->mode = FLIGHTMODE_RATE;

        throttle = RCInputGetInputLevel(ROLE_THROTTLE);
        ref->rate_reference.x =
            control_limits.max_rate.pitch * DEG2RAD * RCInputGetInputLevel(ROLE_PITCH);
        ref->rate_reference.y =
            control_limits.max_rate.roll * DEG2RAD * RCInputGetInputLevel(ROLE_ROLL);
        ref->rate_reference.z =
            control_limits.max_rate.yaw * DEG2RAD * RCInputGetInputLevel(ROLE_YAW);
    }
    else if (selector == FLIGHTMODE_ATTITUDE)
    {
        control_reference.mode = FLIGHTMODE_ATTITUDE;

        throttle = RCInputGetInputLevel(ROLE_THROTTLE);
        ref->attitude_reference.y =
            control_limits.max_angle.pitch * DEG2RAD * RCInputGetInputLevel(ROLE_PITCH);
        ref->attitude_reference.x =
            control_limits.max_angle.roll * DEG2RAD * RCInputGetInputLevel(ROLE_ROLL);
        ref->rate_reference.z =
            control_limits.max_rate.yaw * DEG2RAD * RCInputGetInputLevel(ROLE_YAW);
    }

    ref->actuator_desired.throttle = throttle;
}

