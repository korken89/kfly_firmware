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
// void vInitPIController(pid_data_t *pi_settings,
//                        const float P_gain,
//                        const float I_gain)
// {
//     pi_settings->P_gain = P_gain;
//     pi_settings->I_gain = I_gain;
//     pi_settings->I_state = 0.0f;
// }

/**
 * @brief       Updates the gains of a PI controller.
 *
 * @param[in/out] pi_settings   PI settings ans state structure.
 * @param[in] P_gain            Controller P gain.
 * @param[in] I_gain            Controller I gain.
 * @param[in] I_limit           Controller I limit.
 */
// void vUpdatePISettings(pi_data_t *pi_settings,
//                        const float P_gain,
//                        const float I_gain)
// {
//     pi_settings->P_gain = P_gain;
//     pi_settings->I_gain = I_gain;
// }
//
