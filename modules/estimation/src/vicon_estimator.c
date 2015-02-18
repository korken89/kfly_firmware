/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "vicon_estimator.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/
static vicon_measurement_t vicon_data;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void vInitializeViconEstimator(attitude_states_t *states)
{
    /* Wait while there is no valid data from the Vicon system. */
    do
    {
        chThdSleepMilliseconds(10);
        GetCopyViconMeasurement(&vicon_data);
    } while (vicon_data.frame_number == 0);
}

void vInnovateViconEstimator(attitude_states_t *states,
                             vector3f_t gyro_rate,
                             const float dt,
                             const float gyro_lpf)
{

}
