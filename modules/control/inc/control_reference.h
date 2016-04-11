#ifndef __CONTROL_REFERENCE_H
#define __CONTROL_REFERENCE_H

#include "geometry.h"
#include "rc_input.h"

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
    FLIGHTMODE_DIRECT = 1,
    /**
     * @brief   Direct control of the actuator desired commands.
     */
    FLIGHTMODE_INDIRECT = 2,
    /**
     * @brief   Rate control.
     */
    FLIGHTMODE_RATE = 3,
    /**
     * @brief   Attitude control in Euler angles, used when only controlling
     *          roll and pitch in manual mode.
     */
    FLIGHTMODE_ATTITUDE_EULER = 4,
    /**
     * @brief   Attitude control in quaternion mode, used in computer control.
     */
    FLIGHTMODE_ATTITUDE = 5
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
     * @brief   Control signal to each output channel.
     */
    float output[8];
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
