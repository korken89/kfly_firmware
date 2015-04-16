/* *
 *
 *
 * */

#include "pid.h"
#include "trigonometry.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/
/**
 * @brief       Initializes the state and sets the gains of a PI controller. 
 * 
 * @param[in/out] pi_settings   PI settings ans state structure.
 * @param[in] P_gain            Controller P gain.
 * @param[in] I_gain            Controller I gain.
 * @param[in] I_limit           Controller I limit.
 */
void vInitPIController(PI_Data *pi_settings,
                       float P_gain,
                       float I_gain,
                       float I_limit)
{
    pi_settings->P_gain = P_gain;
    pi_settings->I_gain = I_gain;
    pi_settings->I_limit = I_limit;
    pi_settings->I_state = 0.0f;
}

/**
 * @brief       Updates the gains of a PI controller. 
 * 
 * @param[in/out] pi_settings   PI settings ans state structure.
 * @param[in] P_gain            Controller P gain.
 * @param[in] I_gain            Controller I gain.
 * @param[in] I_limit           Controller I limit.
 */
void vUpdatePISettings(PI_Data *pi_settings,
                       float P_gain,
                       float I_gain,
                       float I_limit)
{
    pi_settings->P_gain = P_gain;
    pi_settings->I_gain = I_gain;
    pi_settings->I_limit = I_limit;
}

/**
 * @brief       Updates the PI controller with one time step.
 * @note        The integral limits directly sets the maximum/minimum of the
 *              integral state.
 * 
 * @param[in/out] pi_settings   PI settings and state structure.
 * @param[in] error             Controlled variable's error.
 * @param[in] dt                Time step size in seconds.
 * @return      The outputted control signal of the PI controller.
 */
float fPIUpdate(PI_Data *pi, float error, float dt)
{
    /* Integration with anti-windup */
    pi->I_state = bound( pi->I_limit,
                        -pi->I_limit,
                         pi->I_state + pi->I_gain * error * dt);

    /* Create control signal */
    return ((pi->P_gain * error) + pi->I_state);
}

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
float fPIUpdate_BackCalculationSaturation(PI_Data *pi,
                                          float error,
                                          float u_max,
                                          float u_min,
                                          float dt)
{
    float u, P_state;

    /* Calculate the new integral state. */
    pi->I_state += dt * pi->I_gain * error;

    /* Calculate the proportional state . */
    P_state = pi->P_gain * error;

    /* Calculate the unsaturated control signal. */ 
    u = P_state + pi->I_state;

    /* Check the saturation and compensate with the integral if necessary. */
    if (u > u_max)
    {
        pi->I_state = u_max - P_state;
        return u_max;
    }
    else if (u < u_min)
    {
        pi->I_state = u_min - P_state;
        return u_min;
    }
    else
    {
        return u;
    }
}
