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
    GetCopyViconMeasurement(&vicon_data);

    /* Wait while there is no valid data from the Vicon system. */
    while (vicon_data.frame_number == 0)
    {
        chThdSleepMilliseconds(20);
        GetCopyViconMeasurement(&vicon_data);
    }

    /* Take the first measurement at starting point. */
    states->q = vicon_data.attitude;

    states->w.x = 0.0f;
    states->w.y = 0.0f;
    states->w.z = 0.0f;

    states->wb.x = 0.0f;
    states->wb.y = 0.0f;
    states->wb.z = 0.0f;
}

void vInnovateViconEstimator(attitude_states_t *states,
                             imu_data_t *imu_data,
                             const float dt,
                             const float gyro_lpf)
{

}
