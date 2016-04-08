/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "motion_capture_estimator.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/
static motion_capture_t mc_data;
static uint32_t old_frame_number;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief               Initialization for the Motion Capture estimator.
 *
 * @param[in] states    Unused.
 */
void vInitializeMotionCaptureEstimator(attitude_states_t *states)
{
    /* New initialization. */
    (void) states;

    states->q = UNIT_QUATERNION;
    old_frame_number = 0;
}

/**
 * @brief               Innovate the Motion Capture estimator.
 *
 * @param[in/out] states    Attitude states to be updated.
 * @param[in] imu_data      Latest IMU measurement.
 * @param[in] imu_dt        Sampling time of the IMU data.
 * @param[in] wb_gain       Gain for the rate bias estimation.
 * @param[in] gyro_lpf      Time constant for the gyro lowpass filter.
 */
void vInnovateMotionCaptureEstimator(attitude_states_t *states,
                                     const imu_data_t *imu_data,
                                     const float imu_dt,
                                     const float wb_gain,
                                     const float gyro_lpf)
{
    (void) wb_gain;
    vector3f_t w_hat; //, wb_step;
    //quaternion_t q_err;

    /* Get the current motion capture data */
    GetCopyMotionCaptureFrame(&mc_data);

    /* Check for startup conditions. */
    if ((mc_data.frame_number > 0) && (old_frame_number == 0))
    {
        /* On the first measurement, set the quaternion to this and reset
         * all other states. */
        states->q = mc_data.pose.orientation;

        states->w.x = 0.0f;
        states->w.y = 0.0f;
        states->w.z = 0.0f;

        states->wb.x = 0.0f;
        states->wb.y = 0.0f;
        states->wb.z = 0.0f;

        old_frame_number = mc_data.frame_number;

        return;
    }

    /* 1. Remove bias from the measurement. */
    w_hat = vector_sub(array_to_vector(imu_data->gyroscope), states->wb);

    /* Check if there was new motion capture data. */
    if (mc_data.frame_number > old_frame_number)
    {
        /* New motion capture data, update the bias and attitude estimation. */
        old_frame_number = mc_data.frame_number;

        /* 2. Integrate the quaternion. */
        //q_err = qint(states->q, w_hat, imu_dt);

        /* 3. Create the error quaternion. */
        //q_err = qmult(qconj(mc_data.pose.orientation), q_err);

        /* 4. Estimate the gyro bias. */
        //wb_step = vector_scale(array_to_vector(&q_err.x), wb_gain / dt);

        /* 5. Apply estimate and update the estimation. */
        //states->wb = vector_add(states->wb, wb_step);
        states->q = mc_data.pose.orientation;
        //w_hat = vector_sub(w_hat, wb_step);
    }
    else
    {
        /* No new motion capture data, use gyros to update the attitude estimation. */

        /* 2. Integrate and save the quaternion, and save the omega. */
        states->q = qint(states->q, w_hat, imu_dt);
    }

    /* 6. Apply low-pass filtering of gyro data and save it. */
    states->w = vector_lowpassfilter(w_hat, states->w, gyro_lpf);
}

