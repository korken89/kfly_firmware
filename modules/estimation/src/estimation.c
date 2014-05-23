/* *
 *
 *
 * */

/* Includes */
#include "ch.h"
#include "hal.h"
#include "estimation.h"
#include "sensor_read.h"

/* Private Defines */

/* Private Typedefs */

/* Global variable defines */	

/* Private function defines */
static THD_FUNCTION(ThreadEstimation, arg);

/* Working area for the estimation thread */
CCM_MEMORY static THD_WORKING_AREA(waThreadEstimation, 512);
CCM_MEMORY static Attitude_Estimation_States states;
CCM_MEMORY static Attitude_Estimation_Data data;
CCM_MEMORY static IMU_Data imu_data;

static thread_t *tp;

/* Private inline functions */
static inline void vector_accumulate(vector3f_t *a, vector3f_t *b)
{
	a->x += b->x;
	a->y += b->y;
	a->z += b->z;
}

/**
 * @brief Initializes all estimation threads.
 */
void EstimationInit(void)
{
	chThdCreateStatic(waThreadEstimation,
                      sizeof(waThreadEstimation), 
                      HIGHPRIO - 1, 
                      ThreadEstimation, 
                      NULL);

}

/**
 * @brief Requests a reset of the estimation.
 */
void ResetEstimation(void)
{
	if (tp != NULL)
		chEvtSignal(tp, ESTIMATION_RESET_EVENT);
}

/**
 * @brief Main attitude estimation thread.
 * 
 * @param[in/out] arg 	Unused.
 */
static THD_FUNCTION(ThreadEstimation, arg)
{
	(void)arg;

	chRegSetThreadName("Estimation");

	tp = chThdGetSelfX();

	/* Initialization data */
	uint32_t i;
	quaternion_t q_init;
	vector3f_t wb_init, acc_init, mag_init;

	/* Event registration for new sensor data */
	event_listener_t el;

	/* Register to new data from accelerometer and gyroscope */
	chEvtRegisterMask(ptrGetNewDataEventSource(),
					  &el,
					  ACCGYRO_DATA_AVAILABLE_EVENTMASK |
					  MAG_DATA_AVAILABLE_EVENTMASK |
					  BARO_DATA_AVAILABLE_EVENTMASK);

	/* Force an initialization of the estimation */
	chEvtAddEvents(ESTIMATION_RESET_EVENT);

	while(1)
	{
		/* Check if there has been a request to reset the filter */
		if (chEvtWaitOneTimeout(ESTIMATION_RESET_EVENT, TIME_IMMEDIATE))
		{
			/* Filter reset requested */

			/* Zero the accumulation vectors */
			wb_init.x = acc_init.x = mag_init.x = 0.0f;
			wb_init.y = acc_init.y = mag_init.y = 0.0f;
			wb_init.z = acc_init.z = mag_init.z = 0.0f;

			/* Sum measurements */
			for (i = 0; i < 50; i++)
			{
				/* Wait for new measurement data using the slowest sensor */	
				chEvtWaitOne(MAG_DATA_AVAILABLE_EVENTMASK);

				/* Get sensor data */
				GetIMUData(&imu_data);

				/* Sum the inertial data */
				vector_accumulate(&acc_init,
								  (vector3f_t *)imu_data.accelerometer);
				vector_accumulate(&wb_init, 
								  (vector3f_t *)imu_data.gyroscope);
				vector_accumulate(&mag_init, 
							      (vector3f_t *)imu_data.magnetometer);
			}

			/* Create mean value */
			acc_init = vector_scale(acc_init, 1.0f / ((float) i));
			mag_init = vector_scale(mag_init, 1.0f / ((float) i));

			/* Generate the starting guess quaternion */
			GenerateStartingGuess(&acc_init, &mag_init, &q_init);
		
			/* Initialize the estimation */
			AttitudeEstimationInit(&states, &data, &q_init, &wb_init);
		}

		/* Wait for new measurement data */	
		chEvtWaitOne(ACCGYRO_DATA_AVAILABLE_EVENTMASK);

		/* Get sensor data */
		GetIMUData(&imu_data);

		/* Runs estimation */
		InnovateAttitudeEKF(&states,
							&data, 
							imu_data.gyroscope,
							imu_data.accelerometer,
							imu_data.magnetometer,
							0.0f,
							0.0f,
							0.005f);
	}

	return MSG_OK;
}
