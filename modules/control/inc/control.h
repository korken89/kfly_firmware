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

#define RATE_LIMIT_OFFSET                       0
#define ATTITUDE_LIMIT_OFFSET                   12
#define VELOCITY_LIMIT_OFFSET                   32
#define POSITION_LIMIT_OFFSET                   40

#define RATE_LIMIT_COUNT                        12
#define ATTITUDE_LIMIT_COUNT                    20
#define VELOCITY_LIMIT_COUNT                    8
#define POSITION_LIMIT_COUNT                    0

/* Sizes */
#define OUTPUT_MIXER_SIZE                       (sizeof(Output_Mixer))
#define CONTROL_ARM_SIZE                        (sizeof(Control_Arm_Settings))
#define CONTROL_LIMITS_SIZE                     (sizeof(Control_Limits))
#define CONTROL_REFERENCE_SIZE                  (sizeof(Control_Reference))
#define CONTROL_NUMBER_OF_CONTROLLERS           (12)
#define CONTROL_DATA_SIZE                       (sizeof(Control_Data))
#define CONTROL_PARAMETERS_SIZE                 (sizeof(Control_Parameters))

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
} Arming_Stick_Direction;

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
} Arming_Stick_Region;

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
    FLIGHTMODE_POSITION_HOLD
} Flight_Mode;

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
} Target_Direcion;

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
    Arming_Stick_Direction stick_direction;
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
} Control_Arm_Settings;

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
    Flight_Mode mode;
    /**
     * @brief   Current targeting scheme for position and velocity control.
     */
    Target_Direcion target;
} Control_Reference;

/**
 * @brief   Position, velocity, attitude and rate controller gains and states.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Position controller gains and states.
     */
    PI_Data position_controller[3];
    /**
     * @brief   Velocity controller gains and states.
     */
    PI_Data velocity_controller[3];
    /**
     * @brief   Attitude controller gains and states.
     */
    PI_Data attitude_controller[3];
    /**
     * @brief   Rate controller gains and states.
     */
    PI_Data rate_controller[3];
} Control_Data;

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
         * @brief   Pitch rate limit in deg/s.
         */
        float pitch;
        /**
         * @brief   Roll rate limit in deg/s.
         */
        float roll;
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
         * @brief   Pitch rate limit in deg/s.
         */
        float pitch;
        /**
         * @brief   Roll rate limit in deg/s.
         */
        float roll;
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
         * @brief   Pitch attitude limit in radians.
         */
        float pitch;
        /**
         * @brief   Roll attitude limit in radians.
         */
        float roll;
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
} Control_Limits;

/**
 * @brief   Output mixer weights.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Weights.
     */
    float weights[8][4];
} Output_Mixer;

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
} PI_Parameters;

/**
 * @brief   Control parameters structure for moving data.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Position controller parameters.
     */
    PI_Parameters position_parameters[3];
    /**
     * @brief   Velocity controller parameters.
     */
    PI_Parameters velocity_parameters[3];
    /**
     * @brief   Attitude controller parameters.
     */
    PI_Parameters attitude_parameters[3];
    /**
     * @brief   Rate controller parameters.
     */
    PI_Parameters rate_parameters[3];
} Control_Parameters;

typedef struct PACKED_VAR
{
    /*
    // Full measurement

    int16_t accelerometer[3];
    int16_t gyroscope[3];
    int16_t magnetometer[3];
    int8_t u_throttle;
    int8_t u_pitch;
    int8_t u_roll;
    int8_t u_yaw;
    uint8_t counter;
    */

    // Accelerometer only test
    int16_t accelerometer[3];
    int8_t u_throttle;
    uint8_t counter;
} Experiment_Data;

typedef struct PACKED_VAR
{
    vector3f_t rate_reference;
    vector3f_t rate_measured;
    struct {
        float pitch;
        float roll;
        float yaw;
        float throttle;
    } actuator_desired;
    struct {
        float m1;
        float m2;
        float m3;
        float m4;
    } pwm_to_motors;
} Control_Debug;

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
Control_Arm_Settings *ptrGetControlArmSettings(void);
Control_Reference *ptrGetControlReferences(void);
Control_Data *ptrGetControlData(void);
Control_Limits *ptrGetControlLimits(void);
Output_Mixer *ptrGetOutputMixer(void);
void GetControlParameters(Control_Parameters *param);
void SetControlParameters(Control_Parameters *param);

#endif