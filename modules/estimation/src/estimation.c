/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "estimation.h"
#include "sensor_read.h"
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
THD_WORKING_AREA(waThreadEstimation, 1024);
attitude_states_t states;
attitude_matrices_t data;
imu_data_t imu_data;
EVENTSOURCE_DECL(estimation_events_es);

static thread_t *tp;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief Main estimation thread.
 *
 * @param[in/out] arg   Unused.
 */
static THD_FUNCTION(ThreadEstimation, arg)
{
    (void)arg;

    chRegSetThreadName("Estimation");

    tp = chThdGetSelfX();

    /* Initialization data */
    //uint32_t i;
    //quaternion_t q_init = {1.0f, 0.0f, 0.0f, 0.0f};
    //vector3f_t am, wb_init = {0.0f, 0.0f, 0.0f}; //, acc_init, mag_init;

    /* Event registration for new sensor data */
    event_listener_t el;

    /* Register to new data from accelerometer and gyroscope */
    chEvtRegisterMask(ptrGetNewDataEventSource(),
                      &el,
                      ACCGYRO_DATA_AVAILABLE_EVENTMASK |
                      MAG_DATA_AVAILABLE_EVENTMASK |
                      BARO_DATA_AVAILABLE_EVENTMASK);

    /* Force an initialization of the estimation */
    //chEvtAddEvents(ESTIMATION_RESET_EVENTMASK);

    //AttitudeEstimationInit(&states, &data, &q_init, &wb_init);
    vInitializeViconEstimator(&states);

    while(1)
    {
        //if (isnan(states.q.q0) || isnan(states.q.q1) || isnan(states.q.q2) || isnan(states.q.q3) ||
        //    isnan(states.w.x) || isnan(states.w.y) || isnan(states.w.z) ||
        //    isnan(states.wb.x) || isnan(states.wb.y) || isnan(states.wb.z))
        //    AttitudeEstimationInit(&states, &data, &q_init, &wb_init);

        /* Check if there has been a request to reset the filter */
        //if (chEvtWaitOneTimeout(ESTIMATION_RESET_EVENTMASK, TIME_IMMEDIATE))
       // {
            /* Initialize the estimation */
            //AttitudeEstimationInit(&states, &data, &q_init, &wb_init);
        //}

        /* Wait for new measurement data */
        chEvtWaitOne(ACCGYRO_DATA_AVAILABLE_EVENTMASK);

        /* Get sensor data */
        GetIMUData(&imu_data);

        /* Run estimation */
        vInnovateViconEstimator(&states,
                                &imu_data,
                                SENSOR_ACCGYRO_DT,
                                0.0005f,
                                fc2lpf_gain(20, SENSOR_ACCGYRO_DT)); /* LPF ~ 45 Hz */

        /*InnovateAttitudeEKF(&states,
                            &data,
                            imu_data.gyroscope,
                            imu_data.accelerometer,
                            imu_data.magnetometer,
                            0.0f,
                            0.0f,
                            ESTIMATION_DT);*/

        //states.w.x = -imu_data.gyroscope[0];
        //states.w.y = -imu_data.gyroscope[1];
        //states.w.z = imu_data.gyroscope[2];

        //am.x = -imu_data.accelerometer[0];
        //am.y = -imu_data.accelerometer[1];
        //am.z = imu_data.accelerometer[2];

        //states.q = MadgwickAHRSupdateIMU(states.w,
        //                                 am,
        //                                 states.q,
        //                                 0.15f,
        //                                 dt);

        /* Broadcast new estimation available */
        chEvtBroadcastFlags(&estimation_events_es,
                            ESTIMATION_NEW_ESTIMATION_EVENTMASK);
    }
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief Initializes all estimation threads.
 */
void EstimationInit(void)
{
    /* Initialize new estimation event source */
    osalEventObjectInit(&estimation_events_es);

    /* Start the estimation thread */
    chThdCreateStatic(waThreadEstimation,
                      sizeof(waThreadEstimation),
                      HIGHPRIO - 2,
                      ThreadEstimation,
                      NULL);

}

/**
 * @brief Requests a reset of the estimation.
 */
void ResetEstimation(void)
{
    if (tp != NULL)
        chEvtSignal(tp, ESTIMATION_RESET_EVENTMASK);
}

/**
 * @brief Returns the pointer to the attitude estimation states.
 *
 * @return Pointer to the attitude estimation states.
 */
attitude_states_t *ptrGetAttitudeEstimationStates(void)
{
    return &states;
}

/**
 * @brief Returns the pointer to the estimation event source.
 *
 * @return Pointer to the estimation event source.
 */
event_source_t *ptrGetEstimationEventSource(void)
{
    return &estimation_events_es;
}

quaternion_t MadgwickAHRSupdateIMU(vector3f_t g,
                                   vector3f_t a,
                                   quaternion_t q,
                                   float beta,
                                   float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1;
    float _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.x * g.x - q.y * g.y - q.z * g.z);
    qDot2 = 0.5f * (q.w * g.x + q.y * g.z - q.z * g.y);
    qDot3 = 0.5f * (q.w * g.y - q.x * g.z + q.z * g.x);
    qDot4 = 0.5f * (q.w * g.z + q.x * g.y - q.y * g.x);

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalization)
    if(!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _4q0 = 4.0f * q.w;
        _4q1 = 4.0f * q.x;
        _4q2 = 4.0f * q.y;
        _8q1 = 8.0f * q.x;
        _8q2 = 8.0f * q.y;
        q0q0 = q.w * q.w;
        q1q1 = q.x * q.x;
        q2q2 = q.y * q.y;
        q3q3 = q.z * q.z;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
        s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q.x - _2q0 * a.y
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
        s2 = 4.0f * q0q0 * q.y + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
        s3 = 4.0f * q1q1 * q.z - _2q1 * a.x + 4.0f * q2q2 * q.z - _2q2 * a.y;
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;

    // Normalize quaternion
    recipNorm = 1.0f / sqrtf(q.w * q.w + q.x * q.x +
                             q.y * q.y + q.z * q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    return q;
}
