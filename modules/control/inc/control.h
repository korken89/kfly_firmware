#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include "trigonometry.h"
#include "quaternion.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define RATE_PI_OFFSET          0
#define ATTITUDE_PI_OFFSET      3
#define VELOCITY_PI_OFFSET      6
#define POSITION_PI_OFFSET      9

#define RATE_LIMIT_OFFSET       0
#define ATTITUDE_LIMIT_OFFSET   12
#define VELOCITY_LIMIT_OFFSET   32
#define POSITION_LIMIT_OFFSET   40

#define RATE_LIMIT_COUNT        12
#define ATTITUDE_LIMIT_COUNT    20
#define VELOCITY_LIMIT_COUNT    8
#define POSITION_LIMIT_COUNT    0

/* Sizes */
#define OUTPUT_MIXER_SIZE       (4*8*4)
#define CONTROL_DATA_SIZE       (12*PI_DATA_SIZE)
#define CONTROL_LIMITS_SIZE     (10*4)
#define CONTROL_REFERENCE_SIZE  (17 * 4 + 2)

/*===========================================================================*/
/* Module data structures and types.                                         */
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
 * @brief   Position, velocity, attitude, rate and actuator
 *          controller references.
 */
typedef struct
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
typedef struct
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
typedef struct
{
    /**
     * @brief   Holder for the rate limits.
     */
    struct
    {
        /**
         * @brief   Pitch rate limit in rad/s.
         */
        float pitch;
        /**
         * @brief   Roll rate limit in rad/s.
         */
        float roll;
        /**
         * @brief   Yaw rate limit in rad/s.
         */
        float yaw;
    } max_rate;
    /**
     * @brief   Holder for the rate limits in attitude mode.
     */
    struct
    {
        /**
         * @brief   Pitch rate limit in rad/s.
         */
        float pitch;
        /**
         * @brief   Roll rate limit in rad/s.
         */
        float roll;
        /**
         * @brief   Yaw rate limit in rad/s.
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
typedef struct
{
    /**
     * @brief   Weights.
     */
    float weights[8][4];
} Output_Mixer;

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
Control_Data *ptrGetControlData(void);
Control_Limits *ptrGetControlLimits(void);
Output_Mixer *ptrGetOutputMixer(void);

#endif