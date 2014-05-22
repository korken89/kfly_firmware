/* *
 *
 *
 * */

/* Includes */
#include "ch.h"
#include "hal.h"
#include "attitude_ekf.h"
#include "estimation.h"
#include "sensor_read.h"

/* Private Defines */

/* Private Typedefs */

/* Global variable defines */	

/* Private function defines */
static THD_FUNCTION(ThreadEstimation, arg);

/* Working area for the estimation thread */
CCM_MEMORY static THD_WORKING_AREA(waThreadEstimation, 256);
CCM_MEMORY static Attitude_Estimation_States states;
CCM_MEMORY static Attitude_Estimation_Data data;
CCM_MEMORY static IMU_Data imu_data;

void EstimationInit(void)
{
	chThdCreateStatic(waThreadEstimation,
                      sizeof(waThreadEstimation), 
                      HIGHPRIO - 1, 
                      ThreadEstimation, 
                      NULL);

}

void ResetEstimation(Attitude_Estimation_States *states,
					 Attitude_Estimation_Data *data)
{
	(void)states;
	(void)data;
//	uint32_t i;
//
//	quaternion_t q_init;
//	vector3f_t wb_init = {0.0f, 0.0f, 0.0f};
//	vector3f_t acc_init = {0.0f, 0.0f, 0.0f};
//	vector3f_t mag_init = {0.0f, 0.0f, 0.0f};
//
//	Sensor_Data_Type *sensor_data;
//
//	sensor_data = ptrGetSensorDataPointer();
//
//
//	/* Take 100 measurements to create a starting guess for the filter */
//	for (i = 0; i < 100; i++)
//	{
//		xSemaphoreTake(NewMeasurementAvaiable, portMAX_DELAY);
//
//		acc_init = vector_add(acc_init, sensor_data->acc);
//		mag_init = vector_add(mag_init, sensor_data->mag);
//	}
//
//	/* Scale all the measurements to get the mean value */
//	acc_init = vector_scale(acc_init, 1.0f / ((float) i));
//	mag_init = vector_scale(mag_init, 1.0f / ((float) i));
//
//	/* Generate the starting guess quaternion */
//	GenerateStartingGuess(&acc_init, &mag_init, &q_init);
//
//	/* Initialize the estimation */
//	AttitudeEstimationInit(states, data, &q_init, &wb_init, 0.005f);
}

static THD_FUNCTION(ThreadEstimation, arg)
{
	(void)arg;

	chRegSetThreadName("Estimation");

	/* Event registration for new sensor data */
	event_listener_t el;
	event_source_t *es;
	eventmask_t acc_gyro_mask, mag_mask;

	/* Register to new data from accelerometer and gyroscope */
	es = ptrGetAccelerometerAndGyroscopeEventSource(&acc_gyro_mask);
	chEvtRegisterMask(es, &el, acc_gyro_mask);

	/* Register to new data from magnetometer */
	es = ptrGetMagnetometerEventSource(&mag_mask);
	chEvtRegisterMask(es, &el, mag_mask);

	while(1)
	{
		/* Wait for new measurement data */	
		chEvtWaitOne(acc_gyro_mask);

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