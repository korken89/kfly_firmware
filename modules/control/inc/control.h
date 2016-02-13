#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include "trigonometry.h"
#include "quaternion.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define DEG2RAD                                 (0.0174532925199433f)

#define ARM_RATE                                10 /* Hz */

#define RATE_PI_OFFSET                          9
#define ATTITUDE_PI_OFFSET                      6
#define VELOCITY_PI_OFFSET                      3
#define POSITION_PI_OFFSET                      0

/* Sizes */
#define OUTPUT_MIXER_SIZE                       (sizeof(output_mixer_t))
#define CONTROL_ARM_SIZE                        (sizeof(control_arm_settings_t))
#define CONTROL_LIMITS_SIZE                     (sizeof(control_limits_t))
#define CONTROL_REFERENCE_SIZE                  (sizeof(control_reference_t))
#define CONTROL_NUMBER_OF_CONTROLLERS           (12)
#define CONTROL_DATA_SIZE                       (sizeof(control_data_t))
#define CONTROL_PARAMETERS_SIZE                 (sizeof(control_parameters_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Possible stick direction for arming the controllers.
 */
typedef enum PACKED_VAR
{
    /**
     * @brief   Arm direction not yet set.
     */
    STICK_NONE = 0,
    /**
     * @brief   Arm at pitch at min.
     */
    STICK_PITCH_MIN,
    /**
     * @brief   Arm at pitch at max.
     */
    STICK_PITCH_MAX,
    /**
     * @brief   Arm at roll at min.
     */
    STICK_ROLL_MIN,
    /**
     * @brief   Arm at roll at max.
     */
    STICK_ROLL_MAX,
    /**
     * @brief   Arm at yaw at min.
     */
    STICK_YAW_MIN,
    /**
     * @brief   Arm at yaw at max.
     */
    STICK_YAW_MAX
} arming_stick_direction_t;

/**
 * @brief   Possible stick regions for arming/disarming the controllers.
 */
typedef enum
{
    /**
     * @brief   Sticks are neither in the arm or disarm position.
     */
    STICK_NO_REGION = 0,
    /**
     * @brief   Sticks are in the arm position.
     */
    STICK_ARM_REGION,
    /**
     * @brief   Sticks are in the disarm position.
     */
    STICK_DISARM_REGION
} arming_stick_region_t;

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
 * @brief   Possible targeting direction for velocity and position control.
 *          This determines where the system will be looking while following
 *          velocity and position commands.
 */
typedef enum PACKED_VAR
{
    /**
     * @brief   Target a fixed reference angle.
     */
    TARGET_FIXED_ANGLE = 0,
    /**
     * @brief   Target the direction of the velocity vector.
     */
    TARGET_VELOCITY_VECTOR,
    /**
     * @brief   Target the goal point.
     */
    TARGET_GOAL,
    /**
     * @brief   Target coordinate.
     */
    TARGET_COORDINATE
} target_direction_t;

/**
 * @brief   Settings for the arm and disarm functionality.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Stick threshold for the arm/disarm logic to react.
     */
    float stick_threshold;
    /**
     * @brief   Minimum throttle when armed (to spin propellers when armed).
     */
    float armed_min_throttle;
    /**
     * @brief   Stick direction to arm the controllers.
     */
    arming_stick_direction_t stick_direction;
    /**
     * @brief   Time (in seconds) needed to hold the sticks in correct
     *          position in order to arm the system.
     */
    uint8_t arm_stick_time;
    /**
     * @brief   Time (in seconds) needed to disarm the controllers if no
     *          throttle has been given.
     */
    uint8_t arm_zero_throttle_timeout;
} control_arm_settings_t;

/**
 * @brief   Position, velocity, attitude, rate and actuator
 *          controller references.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Position control reference.
     */
    vector3f_t position_reference;
    /**
     * @brief   Velocity control reference.
     */
    vector3f_t velocity_reference;
    /**
     * @brief   Attitude control reference.
     */
    vector3f_t attitude_reference;
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
    /**
     * @brief   Current targeting scheme for position and velocity control.
     */
    target_direction_t target;
} control_reference_t;

/**
 * @brief   Position, velocity, attitude and rate controller gains and states.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Position controller gains and states.
     */
    pi_data_t position_controller[3];
    /**
     * @brief   Velocity controller gains and states.
     */
    pi_data_t velocity_controller[3];
    /**
     * @brief   Attitude controller gains and states.
     */
    pi_data_t attitude_controller[3];
    /**
     * @brief   Rate controller gains and states.
     */
    pi_data_t rate_controller[3];
} control_data_t;

/**
 * @brief   Velocity, attitude and rate control limits.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Holder for the rate limits.
     */
    struct
    {
        /**
         * @brief   Roll rate limit in deg/s.
         */
        float roll;
        /**
         * @brief   Pitch rate limit in deg/s.
         */
        float pitch;
        /**
         * @brief   Yaw rate limit in deg/s.
         */
        float yaw;
    } max_rate;
    /**
     * @brief   Holder for the rate limits in attitude mode.
     */
    struct
    {
        /**
         * @brief   Roll rate limit in deg/s.
         */
        float roll;
        /**
         * @brief   Pitch rate limit in deg/s.
         */
        float pitch;
        /**
         * @brief   Yaw rate limit in deg/s.
         */
        float yaw;
    } max_rate_attitude;
    /**
     * @brief   Holder for the attitude limits.
     */
    struct
    {
        /**
         * @brief   Roll attitude limit in radians.
         */
        float roll;
        /**
         * @brief   Pitch attitude limit in radians.
         */
        float pitch;
    } max_angle;
    /**
     * @brief   Holder for the velocity limits.
     */
    struct
    {
        /**
         * @brief   Horizontal velocity limit in m/s.
         */
        float horizontal;
        /**
         * @brief   Vertical velocity limit in m/s.
         */
        float vertical;
    } max_velocity;
} control_limits_t;

/**
 * @brief   Output mixer weights.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Weights.
     */
    float weights[8][4];
    /**
     * @brief   Offsets to compensate for, as an example, the zero of an servo.
     */
    float offset[8];
} output_mixer_t;

/*
 * Data transfer structures
 */

/**
 * @brief   PI controller parameters structure.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Controller proportional gain.
     */
    float P_gain;
    /**
     * @brief   Controller integral gain.
     */
    float I_gain;
    /**
     * @brief   Controller integral limit.
     */
    float I_limit;
} pi_parameters_t;

/**
 * @brief   Control parameters structure for moving data.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Position controller parameters.
     */
    pi_parameters_t position_parameters[3];
    /**
     * @brief   Velocity controller parameters.
     */
    pi_parameters_t velocity_parameters[3];
    /**
     * @brief   Attitude controller parameters.
     */
    pi_parameters_t attitude_parameters[3];
    /**
     * @brief   Rate controller parameters.
     */
    pi_parameters_t rate_parameters[3];
} control_parameters_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void ControlInit(void);
void vUpdateControlAction(quaternion_t *q_m, vector3f_t *omega_m, float dt);
void vControlForceDisarm(uint32_t key);
control_arm_settings_t *ptrGetControlArmSettings(void);
control_reference_t *ptrGetControlReferences(void);
control_data_t *ptrGetControlData(void);
control_limits_t *ptrGetControlLimits(void);
output_mixer_t *ptrGetOutputMixer(void);
void GetControlParameters(control_parameters_t *param);
void SetControlParameters(control_parameters_t *param);

#endif
