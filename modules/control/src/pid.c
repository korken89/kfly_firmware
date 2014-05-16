/* *
 *
 *
 * */

/* Includes */
#include "pid.h"

/* Private Defines */

/* Private Typedefs */

/* Global variable defines */

/* Private function defines */

void vInitPIController(PI_Data_Type *pi_settings, float P_gain, float I_gain, float I_limit)
{
	pi_settings->P_gain = P_gain;
	pi_settings->I_gain = I_gain;
	pi_settings->I_limit = I_limit;
	pi_settings->I_state = 0.0f;
}

void vUpdatePISettings(PI_Data_Type *pi_settings, float P_gain, float I_gain, float I_limit)
{
	pi_settings->P_gain = P_gain;
	pi_settings->I_gain = I_gain;
	pi_settings->I_limit = I_limit;
}

float fPIUpdate(PI_Data_Type *pi, float error, float dt)
{
    /* Integration with anti-windup */
    pi->I_state = bound(pi->I_limit, - pi->I_limit, pi->I_state + error * dt);

    /* Create control signal */
    return ((pi->P_gain * error) + (pi->I_gain * pi->I_state));
}
