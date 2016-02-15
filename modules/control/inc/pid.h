#ifndef __PID_H
#define __PID_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PI_DATA_SIZE        (sizeof(pi_data_t))
#define PI_SETTINGS_SIZE    (2*sizeof(float))
#define PI_NUM_PARAMETERS   (2)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   PI controller data structure.
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
     * @brief   Current controller integral state.
     */
    float I_state;
} pi_data_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief       Updates the PI controller with one time step.
 * @note        The integral is limited using back calculation to not need
 *              need limits on the integral itself.
 *
 * @param[in/out] pi_settings   PI settings and state structure.
 * @param[in] error             Controlled variable's error.
 * @param[in] u_max             Max saturation limit.
 * @param[in] u_min             Min saturation limit.
 * @param[in] dt                Time step size in seconds.
 * @return      The outputted control signal of the PI controller.
 */
static inline float fPIUpdate_BC(pi_data_t *pi,
                                 const float error,
                                 const float u_max,
                                 const float u_min,
                                 const float dt)
{
    float u, P_state;

    /* Calculate the new integral state. */
    pi->I_state += dt * pi->I_gain * error;

    /* Calculate the proportional state. */
    P_state = pi->P_gain * error;

    /* Calculate the unsaturated control signal. */
    u = P_state + pi->I_state;

    /* Check the saturation and compensate with the integral if necessary. */
    if (u > u_max)
    {
        if (pi->I_gain > 0.0f)
            pi->I_state = u_max - P_state;

        return u_max;
    }
    else if (u < u_min)
    {
        if (pi->I_gain > 0.0f)
            pi->I_state = u_min - P_state;

        return u_min;
    }
    else
        return u;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vInitPIController(pi_data_t *pi_settings,
                       const float P_gain,
                       const float I_gain);
void vUpdatePISettings(pi_data_t *pi_settings,
                       const float P_gain,
                       const float I_gain);

#endif
