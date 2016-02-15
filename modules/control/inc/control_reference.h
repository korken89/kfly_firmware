#ifndef __CONTROL_REFERENCE_H
#define __CONTROL_REFERENCE_H

#include "trigonometry.h"
#include "quaternion.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/**
 * @brief   Possible flight modes the controllers support.
 */
typedef enum PACKED_VAR
{
    /**
     * @brief   Disarm the controllers and set outputs to zero.
     */
    FLIGHTMODE_DISARMED = 0,
    /**
     * @brief   Direct control of the PWM outputs.
     */
    FLIGTMODE_DIRECT_PWM,
    /**
     * @brief   Direct control of the actuator desired commands.
     */
    FLIGHTMODE_DIRECT_CONTROL,
    /**
     * @brief   Rate control.
     */
    FLIGHTMODE_RATE,
    /**
     * @brief   Attitude control in Euler angles, used when only controlling
     *          roll and pitch in manual mode.
     */
    FLIGHTMODE_ATTITUDE_EULER,
    /**
     * @brief   Attitude control in quaternion mode, used in computer control.
     */
    FLIGHTMODE_ATTITUDE
} flightmode_t;

/**
 * @brief   Attitude, rate and actuator controller references.
 */
typedef struct
{
    /**
     * @brief   Attitude control reference (quaternion mode).
     */
    quaternion_t attitude_reference;
    /**
     * @brief   Attitude control reference (Euler mode).
     */
    vector3f_t attitude_reference_euler;
    /**
     * @brief   Rate control reference.
     */
    vector3f_t rate_reference;
    /**
     * @brief   Actuator desired holder.
     */
    struct {
        /**
         * @brief   Desired torque around each axis.
         */
        vector3f_t torque;
        /**
         * @brief   Desired throttle.
         */
        float throttle;
    } actuator_desired;
    /**
     * @brief   Control signal to each PWM channel.
     */
    float pwm_out[8];
    /**
     * @brief   Current flight mode.
     */
    flightmode_t mode;
} control_reference_t;

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

void RCInputsToControlAction(control_reference_t *ref,
                             const vector3f_t *rate_lim,
                             const vector3f_t *attitude_lim);

#endif
