#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include "trigonometry.h"
#include "quaternion.h"

/* Defines */
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


/* Typedefs */
typedef enum
{
    /* The different flight modes available */
    FLIGHTMODE_DISARMED = 0,
    FLIGHTMODE_RATE,
    FLIGHTMODE_ATTITUDE,
    FLIGHTMODE_VELOCITY,
    FLIGHTMODE_POSITION,
    FLIGHTMODE_POSITION_HOLD
} Flight_Mode;

typedef enum
{
    /* This determines where the system will be looking
     * while following velocity and position commands */
    TARGET_FIXED_ANGLE = 0,
    TARGET_VELOCITY_VECTOR,
    TARGET_GOAL,
    TARGET_COORDINATE
} Target_Direcion;

typedef struct
{
    /* The general structure containing control reference */
    Flight_Mode mode;
    Target_Direcion target;
    vector3f_t position_reference;
    vector3f_t velocity_reference;
    quaternion_t attitude_reference;
    vector3f_t rate_reference;
    struct {
        float pitch;
        float roll;
        float yaw;
        float throttle;
    } actuator_desired;
} Control_Reference;

typedef struct
{
    /* The controller gains, integral limits and states
     * of each controller in the cascade scheme */
    PI_Data position_controller[3];
    PI_Data velocity_controller[3];
    PI_Data attitude_controller[3];
    PI_Data rate_controller[3];
} Control_Data;

typedef struct
{
    /* The controller limits for rate, angles and velocity */
    struct /* Belongs to the rate controller */
    {   /* Rate control limits */
        float pitch;
        float roll;
        float yaw;
    } max_rate;

    struct /* Belongs to the attitude controller */
    {   /* Rate control limits whilst in attitude mode */
        float pitch;
        float roll;
        float yaw;
    } max_rate_attitude;

    struct /* Belongs to the attitude controller */
    {   /* Attitude control limits */
        float pitch;
        float roll;
    } max_angle;

    struct /* Belongs to the velocity controller */
    {   /* Velocity control limits */
        float horizontal;
        float vertical;
    } max_velocity;
} Control_Limits;

typedef struct
{
    float weights[8][4];
} Output_Mixer;

/* Global variable defines */

/* Global function defines */
void ControlInit(void);
void vUpdateControlAction(Control_Reference *ref, Control_Limits *limits, float dt);
Control_Data *ptrGetControlData(void);
Control_Limits *ptrGetControlLimits(void);
Output_Mixer *ptrGetOutputMixer(void);

#endif
