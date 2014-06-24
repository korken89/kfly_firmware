/* *
 *
 *
 * */

#include <math.h>
#include "attitude_ekf.h"

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
 * @brief   Used to initialize and zero the matrices and states of the EKF
 * 
 * @param[out] states           Pointer to the EKF states.
 * @param[out] data             Pointer to the EKF data holder.
 * @param[in] start_attitude    Pointer to the starting guess of attitude.
 * @param[in] start_bias        Pointer to the starting guess of gyro bias.
 */
void AttitudeEstimationInit(Attitude_Estimation_States *states,
                            Attitude_Estimation_Data *data,
                            quaternion_t *start_attitude,
                            vector3f_t *start_bias)
{
    /* Initialize states */
    states->q.q0 = start_attitude->q0;
    states->q.q1 = start_attitude->q1;
    states->q.q2 = start_attitude->q2;
    states->q.q3 = start_attitude->q3;

    states->w.x = 0.0f;
    states->w.y = 0.0f;
    states->w.z = 0.0f;

    states->wb.x = start_bias->x;
    states->wb.y = start_bias->y;
    states->wb.z = start_bias->z;


    /*
     * Initialize the gain matrices
     */

    /* Cast the data for better looking code, data->Sp[1][1] is now Sp[1][1] */
    float (*Sp)[6] = data->Sp;
    float (*T1)[6] = data->T1;
    float (*Ss)[3] = data->Ss;
    float (*T2)[3] = data->T2;
    float (*T3)[6] = data->T3;

    /* Zero matrices */
    create_zero(&Sp[0][0], 6, 6);
    create_zero(&Ss[0][0], 3, 3);
    create_zero(&T1[0][0], 6, 6);
    create_zero(&T2[0][0], 3, 3);
    create_zero(&T3[0][0], 3, 6);

    /* Set the starting square-root error covariance matrix */
    Sp[0][0] = SP_1;
    Sp[1][1] = SP_1;
    Sp[2][2] = SP_1;
    Sp[3][3] = SP_1;
    Sp[4][4] = SP_1;
    Sp[5][5] = SP_1;
}

/**
 * @brief   Generates a staring guess attitude quaternion for the EKF,
 * 
 * @param[in] acc               Acceleration vector pointing approximating
 *                              gravity.
 * @param[in] mag               Magnetic vector approximating the earth's 
 *                              magnetic field.
 * @param[out] attitude_guess   Output quaternion.
 */
void GenerateStartingGuess(vector3f_t *acc,
                           vector3f_t *mag, 
                           quaternion_t *attitude_guess)
{
    float pitch, roll, yaw;

    /* Generate pitch and roll from the accelerometer reading */
    roll  = atan2f(-acc->y, acc->z);
    pitch = atan2f(acc->x, sqrtf(acc->y * acc->y + acc->z * acc->z));

    /* Generate yaw by compensating for the pitch and roll */
    yaw = atan2f(mag->y * fast_cos(roll) + mag->z * fast_sin(roll), 
                 (mag->x * fast_cos(pitch) + mag->y * fast_sin(pitch) *
                  fast_sin(roll) - mag->z * fast_sin(pitch) * fast_cos(roll)));

    /* Convert angles into quaternion */
    euler2quat(roll, pitch, yaw, attitude_guess);
}

/**
 * @brief           Attitude estimation update
 * @details         An Square-Root Multiplicative Extended Kalman Filter
 *                  (SR-MEKF) based on Generalized Rodriguez Parameters (GRPs).
 *                  It has very large dynamical range due to the square-root
 *                  factors and is written with close to optimal code. If there 
 *                  is no new magnetometer value, insert NULL and the filter 
 *                  will do an update without the magnetometer.
 * 
 * @param states[in/out]  Pointer to structure holding the states.
 * @param data[in/out]    Pointer to structure holding the data
 *                        and temporary matrices.
 * @param gyro[in]        Gyro input.
 * @param acc[in]         Accelerometer input.
 * @param mag[in]         Magnetometer input.
 * @param beta[in]        Mass compensated thrust coefficient.
 * @param u_sum[in]       The sum of squared control signals.
 * @param dt[in]          The size of the time step.
 */
void InnovateAttitudeEKF(Attitude_Estimation_States *states,
                         Attitude_Estimation_Data *data, 
                         float gyro[3],
                         float acc[3],
                         float mag[3],
                         float beta,
                         float u_sum,
                         float dt)
{
    (void)beta;
    (void)u_sum;
    
    float R[3][3];
    float w_norm, dtheta, sdtheta, cdtheta, t1, t2, t3, x_hat[6];
    quaternion_t dq_int;
    vector3f_t w_hat, theta, mag_v, acc_v, mag_B, acc_B, y;

    /* Cast the data for better looking code, data->Sp[1][1] is now Sp[1][1] */
    float (*Sp)[6] = data->Sp;
    float (*T1)[6] = data->T1;
    float (*Ss)[3] = data->Ss;
    float (*T2)[3] = data->T2;
    float (*T3)[6] = data->T3;

    /* Calculate w_hat */
    w_hat.x = gyro[0] - states->wb.x;
    w_hat.y = gyro[1] - states->wb.y;
    w_hat.z = -gyro[2] - states->wb.z;

    /* Calculate the delta quaternion */
    w_norm = sqrtf(w_hat.x * w_hat.x + w_hat.y * w_hat.y + w_hat.z * w_hat.z);
    
    /* Security to prevent division by very small numbers */
    if (fabsf(w_norm) > 1.0E-4)
    {
        dtheta = 0.5f * w_norm * dt;

        sdtheta = fast_sin(dtheta);
        cdtheta = fast_cos(dtheta);

        /* Calculate the integrated quaternion */
        dq_int.q0 = cdtheta;
        dq_int.q1 = sdtheta * (w_hat.x / w_norm);
        dq_int.q2 = sdtheta * (w_hat.y / w_norm);
        dq_int.q3 = sdtheta * (w_hat.z / w_norm);

        /* Use the delta quaternion to produce the current
           estimate of the attitude */
        states->q = qmult(dq_int, states->q);
    }

    /* Calculate dtheta for each axis */
    theta.x = w_hat.x * dt;
    theta.y = w_hat.y * dt;
    theta.z = w_hat.z * dt;

    /* Convert the current quaternion to a DCM */
    q2dcm(R, &states->q);


    /****************************
     *                          *
     *   Prediction Estimate    *
     *                          *
     ****************************/

    /*
     * 1) Estimate the predicted state:
     */  

    /* Since the error dynamics predict a zero, no change is made */


    /* 
     * 2) Estimate the square-root factor of the predicted
     *    error covariance matrix:
     */

    /* Calculate F_k * Sp_k-1|k-1 part of the QR decomposition,
       with (I + F_k) * Sp_k-1|k-1 */
    t1 = Sp[0][0];
    t2 = Sp[0][1];
    t3 = Sp[0][2];

    Sp[0][0] += t2 * theta.z - Sp[0][3] * dt - t3 * theta.y;
    Sp[0][1] += t3 * theta.x - t1 * theta.z - Sp[0][4] * dt;
    Sp[0][2] += t1 * theta.y - Sp[0][5] * dt - t2 * theta.x;

    t2 = Sp[1][1];
    t3 = Sp[1][2];

    Sp[1][0] += t2 * theta.z - Sp[1][3] * dt - t3 * theta.y;
    Sp[1][1] += t3 * theta.x - Sp[1][4] * dt;
    Sp[1][2] += -Sp[1][5] * dt - t2 * theta.x;

    t3 = Sp[2][2];

    Sp[2][0] += -Sp[2][3] * dt - t3 * theta.y;
    Sp[2][1] += t3 * theta.x - Sp[2][4] * dt;
    Sp[2][2] += -Sp[2][5] * dt;

    Sp[3][0] += -Sp[3][3] * dt;
    Sp[3][1] += -Sp[3][4] * dt;
    Sp[3][2] += -Sp[3][5] * dt;

    Sp[4][1] += -Sp[4][4] * dt;
    Sp[4][2] += -Sp[4][5] * dt;

    Sp[5][2] += -Sp[5][5] * dt;

    /* Copy the Sq values to the temporary matrix
       for use in the decomposition */
    T1[0][0] = SQ_1;
    T1[1][1] = SQ_1;
    T1[2][2] = SQ_1;

    T1[3][3] = SQ_2;
    T1[4][4] = SQ_2;
    T1[5][5] = SQ_2;

    T1[0][3] = SQ_3;
    T1[1][4] = SQ_3;
    T1[2][5] = SQ_3;

    T1[0][1] = 0.0f;
    T1[0][2] = 0.0f;
    T1[0][4] = 0.0f;
    T1[0][5] = 0.0f;
    T1[1][2] = 0.0f;
    T1[1][3] = 0.0f;
    T1[1][5] = 0.0f;
    T1[2][3] = 0.0f;
    T1[2][4] = 0.0f;
    T1[3][4] = 0.0f;
    T1[3][5] = 0.0f;
    T1[4][5] = 0.0f;

    /* Perform the QR decomposition: Sp_k|k-1 = QR([F_k * Sp_k-1|k-1, Sq]^T) */
    qr_decomp_tria(&Sp[0][0], 6);


    /****************************
     *                          *
     *    Measurement update    *
     *                          *
     ****************************/


    /*
     * 1) Subtract the predicted measurement from the true measurement:
     */

    /* Create the measurements */
    acc_v.x = -acc[0];
    acc_v.y = -acc[1];
    acc_v.z = acc[2];

    mag_v.x = mag[0];
    mag_v.y = mag[1];   
    mag_v.z = mag[2];

    acc_B = vector_rotation_transposed(R, acc_v);
    mag_B = vector_rotation_transposed(R, mag_v);

    /* Since the measurement prediction is based on the states and the 
     * states are zero, then the measurement prediction is zero and the
     * error is the measurement directly.
     *
     * The errors are also bounded, in case a measurement makes the
     * division go towards infinity. This should not happen after
     * convergence, but is an added security.
     */
    y.x = atan2f(-acc_B.y, acc_B.z);
    y.y = atan2f( acc_B.x, acc_B.z);
    y.z = atan2f( mag_B.y, mag_B.x);


    /*
     * 2) Estimate the square-root factor of the innovation covariance matrix:
     */
    Ss[0][0] = R[0][0] * Sp[0][0] + R[1][0] * Sp[0][1] + R[2][0] * Sp[0][2]; 
    Ss[0][1] = R[0][1] * Sp[0][0] + R[1][1] * Sp[0][1] + R[2][1] * Sp[0][2]; 
    Ss[0][2] = R[0][2] * Sp[0][0] + R[1][2] * Sp[0][1] + R[2][2] * Sp[0][2]; 

    Ss[1][0] = R[1][0] * Sp[1][1] + R[2][0] * Sp[1][2]; 
    Ss[1][1] = R[1][1] * Sp[1][1] + R[2][1] * Sp[1][2]; 
    Ss[1][2] = R[1][2] * Sp[1][1] + R[2][2] * Sp[1][2]; 

    Ss[2][0] = R[2][0] * Sp[2][2]; 
    Ss[2][1] = R[2][1] * Sp[2][2]; 
    Ss[2][2] = R[2][2] * Sp[2][2];

    /* In the T2 matrix I put the observation covariance matrix. */
    T2[0][0] = SR_1; 
    T2[1][1] = SR_1;
    T2[2][2] = SR_2;

    T2[0][1] = 0.0f;
    T2[0][2] = 0.0f;
    T2[1][2] = 0.0f;

    /* Perform the QR decomposition : Ss_k = QR([H_k * Sp_k|k-1, Sr]^T) */
    qr_decomp_tria(&Ss[0][0], 3);

    /* Invert Ss, since we only need the inverted
       version for future calculations */
    u_inv(&Ss[0][0], 3);

    /* Create T3 = Ss^-1 * H * Sp */
    t1 = R[0][0] * Ss[0][0];
    t2 = R[1][0] * Ss[0][0];
    t3 = R[2][0] * Ss[0][0];

    T3[0][0] = Sp[0][0] * t1 + Sp[0][1] * t2 + Sp[0][2] * t3;
    T3[0][1] = Sp[1][1] * t2 + Sp[1][2] * t3;
    T3[0][2] = Sp[2][2] * t3;

    t1 = R[0][0] * Ss[0][1] + R[0][1] * Ss[1][1];
    t2 = R[1][0] * Ss[0][1] + R[1][1] * Ss[1][1];
    t3 = R[2][0] * Ss[0][1] + R[2][1] * Ss[1][1];

    T3[1][0] = Sp[0][0] * t1 + Sp[0][1] * t2 + Sp[0][2] * t3;
    T3[1][1] = Sp[1][1] * t2 + Sp[1][2] * t3;
    T3[1][2] = Sp[2][2] * t3;

    t1 = R[0][0] * Ss[0][2] + R[0][1] * Ss[1][2] + R[0][2] * Ss[2][2];
    t2 = R[1][0] * Ss[0][2] + R[1][1] * Ss[1][2] + R[1][2] * Ss[2][2];
    t3 = R[2][0] * Ss[0][2] + R[2][1] * Ss[1][2] + R[2][2] * Ss[2][2];

    T3[2][0] = Sp[0][0] * t1 + Sp[0][1] * t2 + Sp[0][2] * t3;
    T3[2][1] = Sp[1][1] * t2 + Sp[1][2] * t3;
    T3[2][2] = Sp[2][2] * t3;


    /*
     * 3) Calculate the Kalman gain, 4) Calculate the updated state: 
     */

    /* K     = Sp * T3^T * Ss^-1 
     * x_hat = K * y
     */
    t1 = Sp[0][0] * T3[0][0];
    t2 = Sp[0][0] * T3[1][0];
    t3 = Sp[0][0] * T3[2][0];

    x_hat[0]  = (Ss[0][0] * t1 + Ss[0][1] * t2 + Ss[0][2] * t3) * y.x;
    x_hat[0] +=                 (Ss[1][1] * t2 + Ss[1][2] * t3) * y.y;
    x_hat[0] +=                                 (Ss[2][2] * t3) * y.z;

    t1 = Sp[0][1] * T3[0][0] + Sp[1][1] * T3[0][1];
    t2 = Sp[0][1] * T3[1][0] + Sp[1][1] * T3[1][1];
    t3 = Sp[0][1] * T3[2][0] + Sp[1][1] * T3[2][1];

    x_hat[1]  = (Ss[0][0] * t1 + Ss[0][1] * t2 + Ss[0][2] * t3) * y.x;
    x_hat[1] +=                 (Ss[1][1] * t2 + Ss[1][2] * t3) * y.y;
    x_hat[1] +=                                 (Ss[2][2] * t3) * y.z;

    t1 = Sp[0][2] * T3[0][0] + Sp[1][2] * T3[0][1] + Sp[2][2] * T3[0][2];
    t2 = Sp[0][2] * T3[1][0] + Sp[1][2] * T3[1][1] + Sp[2][2] * T3[1][2];
    t3 = Sp[0][2] * T3[2][0] + Sp[1][2] * T3[2][1] + Sp[2][2] * T3[2][2];

    x_hat[2]  = (Ss[0][0] * t1 + Ss[0][1] * t2 + Ss[0][2] * t3) * y.x;
    x_hat[2] +=                 (Ss[1][1] * t2 + Ss[1][2] * t3) * y.y;
    x_hat[2] +=                                 (Ss[2][2] * t3) * y.z;

    t1 = Sp[0][3] * T3[0][0] + Sp[1][3] * T3[0][1] + Sp[2][3] * T3[0][2];
    t2 = Sp[0][3] * T3[1][0] + Sp[1][3] * T3[1][1] + Sp[2][3] * T3[1][2];
    t3 = Sp[0][3] * T3[2][0] + Sp[1][3] * T3[2][1] + Sp[2][3] * T3[2][2];

    x_hat[3]  = (Ss[0][0] * t1 + Ss[0][1] * t2 + Ss[0][2] * t3) * y.x;
    x_hat[3] +=                 (Ss[1][1] * t2 + Ss[1][2] * t3) * y.y;
    x_hat[3] +=                                 (Ss[2][2] * t3) * y.z;

    t1 = Sp[0][4] * T3[0][0] + Sp[1][4] * T3[0][1] + Sp[2][4] * T3[0][2];
    t2 = Sp[0][4] * T3[1][0] + Sp[1][4] * T3[1][1] + Sp[2][4] * T3[1][2];
    t3 = Sp[0][4] * T3[2][0] + Sp[1][4] * T3[2][1] + Sp[2][4] * T3[2][2];

    x_hat[4]  = (Ss[0][0] * t1 + Ss[0][1] * t2 + Ss[0][2] * t3) * y.x;
    x_hat[4] +=                 (Ss[1][1] * t2 + Ss[1][2] * t3) * y.y;
    x_hat[4] +=                                 (Ss[2][2] * t3) * y.z;

    t1 = Sp[0][5] * T3[0][0] + Sp[1][5] * T3[0][1] + Sp[2][5] * T3[0][2];
    t2 = Sp[0][5] * T3[1][0] + Sp[1][5] * T3[1][1] + Sp[2][5] * T3[1][2];
    t3 = Sp[0][5] * T3[2][0] + Sp[1][5] * T3[2][1] + Sp[2][5] * T3[2][2];

    x_hat[5]  = (Ss[0][0] * t1 + Ss[0][1] * t2 + Ss[0][2] * t3) * y.x;
    x_hat[5] +=                 (Ss[1][1] * t2 + Ss[1][2] * t3) * y.y;
    x_hat[5] +=                                 (Ss[2][2] * t3) * y.z;


    /*
     * 5) Calculate the square-root factor of the corresponding
     *    error covariance matrix:
     */

    /* Create an 6x6 upper triangular identity matrix */
    create_identity_tria(&T1[0][0], 6);

    /* Perform the Cholesky downdate with each column of T3
       as the downdating vector */
    chol_downdate(&T1[0][0], &T3[0][0], 6);
    chol_downdate(&T1[0][0], &T3[1][0], 6);
    chol_downdate(&T1[0][0], &T3[2][0], 6);

    /* Create the updated error covariance matrix Sp = T1 * Sp (the 
       chol_downdate creates an upper triangular matrix, no transpose needed) */
    uu_mul(&T1[0][0], &Sp[0][0], 6);


    /*
     * 6) Apply the error states to the estimate
     */
    theta.x = x_hat[0];
    theta.y = x_hat[1];
    theta.z = x_hat[2];

    dq_int = grp2q(theta, GRP_A, GRP_F);

    /* Use the delta quaternion to produce the current
       estimate of the attitude */
    states->q = qmult(dq_int, states->q);
    states->q = qnormalize(states->q);

    /* Update the estimation of the bias */
    states->wb.x += x_hat[3];
    states->wb.y += x_hat[4];
    states->wb.z += x_hat[5];

    /* Save the current estimated angular rate */
    states->w.x = gyro[0] - states->wb.x;
    states->w.y = gyro[1] - states->wb.y;
    states->w.z = -gyro[2] - states->wb.z;

    /*
     *      End of filter!
     */ 
}
