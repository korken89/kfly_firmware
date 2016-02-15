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
     * @brief   Attitude control.
     */
    FLIGHTMODE_ATTITUDE,
    /**
     * @brief   Velocity control.
     */
    FLIGHTMODE_VELOCITY,
    /**
     * @brief   Position control.
     */
    FLIGHTMODE_POSITION,
    /**
     * @brief   Position control with position hold.
     */
    FLIGHTMODE_POSITION_HOLD,
    /**
     * @brief   Position control with position hold.
     */
    FLIGHTMODE_COMPUTER_CONTROL
} flightmode_t;

/**
 * @brief   Attitude, rate and actuator controller references.
 */
typedef struct
{
    /**
     * @brief   Attitude control reference.
     */
    quaternion_t attitude_reference;
    /**
     * @brief   Rate control reference.
     */
    vector3f_t rate_reference;
    /**
     * @brief   Actuator desired holder.
     */
    struct {
        /**
         * @brief   Desired pitch torque.
         */
        float pitch;
        /**
         * @brief   Desired roll torque.
         */
        float roll;
        /**
         * @brief   Desired yaw torque.
         */
        float yaw;
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

void UpdateRCInputsToControlAction(void);
control_reference_t *ptrGetControlReferences(void);

#endif
