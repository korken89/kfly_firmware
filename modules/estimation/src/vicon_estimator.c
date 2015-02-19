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
static uint32_t old_frame_number;

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
    while ((vicon_data.available_data & VICON_QUATERNION_MASK) == 0)
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

    old_frame_number = vicon_data.frame_number;
}

void vInnovateViconEstimator(attitude_states_t *states,
                             imu_data_t *imu_data,
                             const float dt,
                             const float wb_gain,
                             const float gyro_lpf)
{
    (void)gyro_lpf;

    vector3f_t w_hat, wb_step;
    quaternion_t q_err;


    /* Get the current Vicon data */
    GetCopyViconMeasurement(&vicon_data);

    /* 1. Remove bias from the measurement. */
    w_hat = vector_sub(array_to_vector(imu_data->gyroscope), states->wb);

    /* Check if there was new Vicon data. */
    if ((vicon_data.frame_number > old_frame_number) &&
        (vicon_data.available_data & VICON_QUATERNION_MASK))
    {
        /* New Vicon data, update the bias and attitude estimation. */
        old_frame_number = vicon_data.frame_number;

        /* 2. Integrate the quaternion. */
        q_err = qint(states->q, w_hat, dt);

        /* 3. Create the error quaternion. */
        q_err = qmult(q_err, qconj(vicon_data.attitude));

        /* 4. Estimate the gyro bias. */
        wb_step = vector_scale(array_to_vector(&q_err.q1), wb_gain);

        /* 5. Apply estimate and update the estimation. */
        states->wb = vector_add(states->wb, wb_step);
        states->q = vicon_data.attitude;
        w_hat = vector_sub(w_hat, wb_step);
    }
    else
    {
        /* No new Vicon data, use gyros to update the attitude estimation. */

        /* 2. Integrate and save the quaternion, and save the omega. */
        states->q = qint(states->q, w_hat, dt);
    }

    /* 6. Apply low-pass filtering of gyro data and save it. */



    /* 7. Save the omega. */
    states->w = w_hat;
}
