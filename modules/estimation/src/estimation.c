/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "estimation.h"
#include "sensor_read.h"

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
Attitude_Estimation_States states;
Attitude_Estimation_Data data;
IMU_Data imu_data;
EVENTSOURCE_DECL(estimation_events_es);

static thread_t *tp;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/
/**
 * @brief       Accumulates (sums) vectors.
 * 
 * @param[in/out] a     Accumulation vector.
 * @param[in] b         Vector to be accumulated.
 */
static inline void vector_accumulate(vector3f_t *a, vector3f_t *b)
{
    a->x += b->x;
    a->y += b->y;
    a->z += b->z;
}

time_measurement_t mytm;
float dt;
/**
 * @brief Main estimation thread.
 * 
 * @param[in/out] arg   Unused.
 */
__attribute__((noreturn))
static THD_FUNCTION(ThreadEstimation, arg)
{
    (void)arg;

    chRegSetThreadName("Estimation");

    tp = chThdGetSelfX();

    /* Initialization data */
    //uint32_t i;
    quaternion_t q_init = {1.0f, 0.0f, 0.0f, 0.0f};
    vector3f_t am, wb_init = {0.0f, 0.0f, 0.0f}; //, acc_init, mag_init;
    bool first_time = true;


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
    AttitudeEstimationInit(&states, &data, &q_init, &wb_init);

    while(1)
    {
        if (isnan(states.q.q0) || isnan(states.q.q1) || isnan(states.q.q2) || isnan(states.q.q3) ||
            isnan(states.w.x) || isnan(states.w.y) || isnan(states.w.z) ||
            isnan(states.wb.x) || isnan(states.wb.y) || isnan(states.wb.z))
            AttitudeEstimationInit(&states, &data, &q_init, &wb_init);
        
        /* Check if there has been a request to reset the filter */
        if (chEvtWaitOneTimeout(ESTIMATION_RESET_EVENTMASK, TIME_IMMEDIATE))
        {
            /* Initialize the estimation */
            AttitudeEstimationInit(&states, &data, &q_init, &wb_init);
        }

        /* Wait for new measurement data */ 
        chEvtWaitOne(ACCGYRO_DATA_AVAILABLE_EVENTMASK);

        if (first_time == true) {
            /* Initialize the measurements */
            first_time = false;
            chTMObjectInit(&mytm);
            chTMStartMeasurementX(&mytm);
        } 
        else
        {
            /* Stop and read the time */
            chTMStopMeasurementX(&mytm);

            /* Some macro to extract the time */
            dt = (float)RTC2US(168000000UL, mytm.last) / 1000000.0f;

            /* Restart the measurement */
            chTMStartMeasurementX(&mytm);


            /* Get sensor data */
            GetIMUData(&imu_data);

            /* Runs estimation */
            /*InnovateAttitudeEKF(&states,
                            &data, 
                            imu_data.gyroscope,
                            imu_data.accelerometer,
                            imu_data.magnetometer,
                            0.0f,
                            0.0f,
                            ESTIMATION_DT);*/

            states.w.x = -imu_data.gyroscope[0];
            states.w.y = -imu_data.gyroscope[1];
            states.w.z = imu_data.gyroscope[2];
    
            am.x = -imu_data.accelerometer[0];
            am.y = -imu_data.accelerometer[1];
            am.z = imu_data.accelerometer[2];
    
            states.q = MadgwickAHRSupdateIMU(states.w,
                                             am,
                                             states.q,
                                             0.15f,
                                             dt);
    
            /* Broadcast new estimation available */
            osalSysLock();
    
            if (chEvtIsListeningI(&estimation_events_es))
            {
                chEvtBroadcastFlagsI(&estimation_events_es,
                                     ESTIMATION_NEW_ESTIMATION_EVENTMASK);
    
                /* osalOsRescheduleS() must be called after a
                   chEvtBroadcastFlagsI() */
                osalOsRescheduleS();
            }

            osalSysUnlock();
        }
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
Attitude_Estimation_States *ptrGetAttitudeEstimationStates(void)
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
    qDot1 = 0.5f * (-q.q1 * g.x - q.q2 * g.y - q.q3 * g.z);
    qDot2 = 0.5f * (q.q0 * g.x + q.q2 * g.z - q.q3 * g.y);
    qDot3 = 0.5f * (q.q0 * g.y - q.q1 * g.z + q.q3 * g.x);
    qDot4 = 0.5f * (q.q0 * g.z + q.q1 * g.y - q.q2 * g.x);

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalization)
    if(!((a.x == 0.0f) && (a.y == 0.0f) && (a.z == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        a.x *= recipNorm;
        a.y *= recipNorm;
        a.z *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.q0;
        _2q1 = 2.0f * q.q1;
        _2q2 = 2.0f * q.q2;
        _2q3 = 2.0f * q.q3;
        _4q0 = 4.0f * q.q0;
        _4q1 = 4.0f * q.q1;
        _4q2 = 4.0f * q.q2;
        _8q1 = 8.0f * q.q1;
        _8q2 = 8.0f * q.q2;
        q0q0 = q.q0 * q.q0;
        q1q1 = q.q1 * q.q1;
        q2q2 = q.q2 * q.q2;
        q3q3 = q.q3 * q.q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
        s1 = _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q.q1 - _2q0 * a.y
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
        s2 = 4.0f * q0q0 * q.q2 + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
        s3 = 4.0f * q1q1 * q.q3 - _2q1 * a.x + 4.0f * q2q2 * q.q3 - _2q2 * a.y;
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
    q.q0 += qDot1 * dt;
    q.q1 += qDot2 * dt;
    q.q2 += qDot3 * dt;
    q.q3 += qDot4 * dt;

    // Normalize quaternion
    recipNorm = 1.0f / sqrtf(q.q0 * q.q0 + q.q1 * q.q1 +
                             q.q2 * q.q2 + q.q3 * q.q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;

    return q;
}
