#ifndef __PID_H
#define __PID_H

#include "ch.h"
#include "biquad.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PID_DATA_SIZE             (sizeof(pid_data_t))
#define PID_PARAMETERS_SIZE       (sizeof(pid_parameters_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef struct PACKED_VAR
{
    /**
     * @brief   Controller proportional gain.
     */
    float P;
    /**
     * @brief   Controller integral gain.
     */
    float I;
    /**
     * @brief   Controller derivative gain.
     */
    float D;
} pid_parameters_t;

/**
 * @brief   PID controller data structure.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Controller gains.
     */
    pid_parameters_t gains;
    /**
     * @brief   Current controller integral state.
     */
    float I_state;
    /**
     * @brief   Old error (for the derivative).
     */
    float error_old;
} pid_data_t;



/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief       Updates the PI controller with one time step.
 * @note        The integral is limited using back calculation to not need
 *              need limits on the integral itself, it is generally a better
 *              integrator limiter than fixed limits.
 *
 * @param[in/out] pid           PID settings and state structure.
 * @param[in/out] dterm_filter  Filter for the derivative term.
 * @param[in] error             Controlled variable's error.
 * @param[in] u_max             Max saturation limit.
 * @param[in] u_min             Min saturation limit.
 * @param[in] dt                Time step size in seconds.
 * @return      The outputted control signal of the PI controller.
 */
static inline float fPIDUpdate_BC(pid_data_t *pid,
                                  biquad_df2t_t *dterm_filter,
                                  const float error,
                                  const float u_max,
                                  const float u_min,
                                  const float dt)
{
    /* Calculate the new integral state. */
    pid->I_state += dt * pid->gains.I * error;

    /* Calculate the new derivative state. */
    float D_state = pid->gains.D * (error - pid->error_old) / dt;
    pid->error_old = error;

    /* Apply filter on D state */
    if (dterm_filter != NULL)
      D_state = BiquadDF2TApply(dterm_filter, D_state);

    /* Calculate the proportional state. */
    const float P_state = pid->gains.P * error;

    /* Calculate the unsaturated control signal. */
    const float u = P_state + pid->I_state + D_state;

    /* Check the saturation and compensate with the integral if necessary. */
    if (u > u_max)
    {
        if (pid->gains.I > 0.0f)
            pid->I_state = u_max - P_state - D_state;

        return u_max;
    }
    else if (u < u_min)
    {
        if (pid->gains.I > 0.0f)
            pid->I_state = u_min - P_state - D_state;

        return u_min;
    }
    else
        return u;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
// void vInitPIController(pid_data_t *pid_settings,
//                        const float P_gain,
//                        const float I_gain,
//                        const float D_gain);
// void vUpdatePISettings(pid_data_t *pid_settings,
//                        const float P_gain,
//                        const float I_gain,
//                        const float D_gain);

#endif
