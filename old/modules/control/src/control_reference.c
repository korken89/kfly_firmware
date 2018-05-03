/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "control_reference.h"
#include "arming.h"
#include "rc_input.h"
#include "math.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static inline float ApplyRateLimitsAndExponentials(const float center_rate,
                                                   const float total_rate,
                                                   const float input)
{
  // Rate exponentials designed to have controlled center linear rate, while
  // having "exponential" kick in outside a small center span (about +/- 30%).
  //
  //
  // Matlab test code:
  //
  // rate_max = 1000;
  // center_rate = 200;
  //
  // x = linspace(-1,1,1000);
  //
  // % Create rate curve
  // linear_rate = x * center_rate;
  // super_rate = x.^3 .* abs(x) * (rate_max - center_rate);
  // y = linear_rate + super_rate;
  //
  // % Plot
  // plot(x,y)
  // grid on
  // xlabel('Normalized input')
  // ylabel('Output rate')

   return input * center_rate +
          fabsf(input) * input * input * input * (total_rate - center_rate);
}

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
                             const control_limits_t *lim)
{
    /* Get the zero integrals function.  */
    void vZeroControlIntegrals(void);

    /* Read out the throttle reference and check if it is bellow the minimum
     * throttle. Used to indicate an armed system by rotating the propellers. */
    float throttle = RCInputGetInputLevel(RCINPUT_ROLE_THROTTLE);

    if (throttle < fGetArmedMinThrottle())
    {
        throttle = fGetArmedMinThrottle();

        /* If the throttle is bellow the minimum armed throttle, zero integrals
         * so there is no windup problem. */
        vZeroControlIntegrals();
    }

    if (ref->mode == FLIGHTMODE_RATE)
    {
      ref->rate_reference.x = ApplyRateLimitsAndExponentials(
          lim->max_rate.center_rate.x, lim->max_rate.max_rate.x,
          RCInputGetInputLevel(RCINPUT_ROLE_ROLL));
      ref->rate_reference.y = ApplyRateLimitsAndExponentials(
          lim->max_rate.center_rate.y, lim->max_rate.max_rate.y,
          RCInputGetInputLevel(RCINPUT_ROLE_PITCH));
      ref->rate_reference.z = ApplyRateLimitsAndExponentials(
          lim->max_rate.center_rate.z, lim->max_rate.max_rate.z,
          RCInputGetInputLevel(RCINPUT_ROLE_YAW));

      ref->actuator_desired.throttle = throttle;
    }
    else if (ref->mode == FLIGHTMODE_ATTITUDE_EULER)
    {
        ref->attitude_reference_euler.x =
            lim->max_angle.roll * RCInputGetInputLevel(RCINPUT_ROLE_ROLL);
        ref->attitude_reference_euler.y =
            lim->max_angle.pitch * RCInputGetInputLevel(RCINPUT_ROLE_PITCH);
        ref->rate_reference.z = ApplyRateLimitsAndExponentials(
            lim->max_rate.center_rate.z, lim->max_rate.max_rate.z,
            RCInputGetInputLevel(RCINPUT_ROLE_YAW));

        ref->actuator_desired.throttle = throttle;
    }
    else
    {
        /* The other modes (quaternion etc) are not supported here yet. */
        ref->actuator_desired.throttle = 0.0f;
    }
}
