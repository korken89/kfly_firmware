#ifndef __CONTROL_DEFINITIONS_H
#define __CONTROL_DEFINITIONS_H

#include "pid.h"
#include "trigonometry.h"
#include "vector3.h"
#include "quaternion.h"
#include "biquad.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define OUTPUT_MIXER_SIZE                       (sizeof(output_mixer_t))
#define CONTROL_ARM_SIZE                        (sizeof(control_arm_settings_t))
#define CONTROL_LIMITS_SIZE                     (sizeof(control_limits_t))
#define CONTROL_REFERENCE_SIZE                  (sizeof(control_reference_t))
#define CONTROL_DATA_SIZE                       (sizeof(control_data_t))
#define CONTROL_PARAMETERS_SIZE                 (sizeof(control_parameters_t))
#define CONTROL_FILTER_SETTINGS_SIZE            (sizeof(control_filter_settings_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/


/**
 * @brief   Position, velocity, attitude and rate controller gains and states.
 */
typedef struct
{
    /**
     * @brief   Attitude controller gains and states.
     */
    pid_data_t attitude_controller[3];
    /**
     * @brief   Rate controller gains and states.
     */
    pid_data_t rate_controller[3];
} control_data_t;

/**
 * @brief   Velocity, attitude and rate control limits.
 */
typedef struct
{
    /**
     * @brief   Holder for the rate limits which builds up the exponential
     *          response in manual mode and limits in automatic mode.
     *
     * @note    max_rate >= center_rate
     * @note    Manual rate = stick * center_rate +
     *                        stick^3 * (max_rate - center_rate)
     */
    struct
    {
        /**
         * @brief   Holder for the rate limits in rad/s.
         */
        vector3f_t max_rate;
        /**
         * @brief   Holder for the center linear rate in rad/s.
         */
        vector3f_t center_rate;
    } max_rate;
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
typedef struct
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
 * @brief   Control parameters structure for moving data.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Attitude controller parameters.
     */
    pid_parameters_t attitude_parameters[3];
    /**
     * @brief   Rate controller parameters.
     */
    pid_parameters_t rate_parameters[3];
    /**
     * @brief   D-term filter cutoff
     */
    float dterm_cutoff[3];
    /**
     * @brief   D-term filter type
     */
    biquad_mode_t dterm_filter_mode[3];

} control_parameters_t;

/**
 * @brief   Control filter settings structure.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   D-term filter cutoff (used for transferring settings)
     */
    float dterm_cutoff[3];
    /**
     * @brief   D-term filter type (used for transferring settings)
     */
    biquad_mode_t dterm_filter_mode[3];
} control_filter_settings_t;

/**
 * @brief   Control filters structure.
 */
typedef struct
{
    /**
     * @brief   D-term filters
     */
    biquad_df2t_t dterm_biquads[3];
    /**
     * @brief   D-term filter settings
     */
    control_filter_settings_t settings;
} control_filters_t;




/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif
