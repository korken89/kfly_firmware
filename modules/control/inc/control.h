#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include "trigonometry.h"
#include "quaternion.h"
#include "control_reference.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/


#define RATE_PI_OFFSET                          3
#define ATTITUDE_PI_OFFSET                      0

/* Sizes */
#define OUTPUT_MIXER_SIZE                       (sizeof(output_mixer_t))
#define CONTROL_ARM_SIZE                        (sizeof(control_arm_settings_t))
#define CONTROL_LIMITS_SIZE                     (sizeof(control_limits_t))
#define CONTROL_REFERENCE_SIZE                  (sizeof(control_reference_t))
#define CONTROL_NUMBER_OF_CONTROLLERS           (6)
#define CONTROL_DATA_SIZE                       (sizeof(control_data_t))
#define CONTROL_PARAMETERS_SIZE                 (sizeof(control_parameters_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Position, velocity, attitude and rate controller gains and states.
 */
typedef struct PACKED_VAR
{
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
    vector3f_t max_rate;
    /**
     * @brief   Holder for the rate limits in attitude mode.
     */
    vector3f_t max_rate_attitude;
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
control_reference_t *ptrGetControlReferences(void);
control_data_t *ptrGetControlData(void);
control_limits_t *ptrGetControlLimits(void);
output_mixer_t *ptrGetOutputMixer(void);
void GetControlParameters(control_parameters_t *param);
void SetControlParameters(control_parameters_t *param);

#endif
