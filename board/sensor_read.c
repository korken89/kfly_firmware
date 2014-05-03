/* *
 *
 * Sensor readout handling
 *
 * */

#include "ch.h"
#include "hal.h"
#include "board.h"
#include "mpu6050.h"
#include "hmc5983.h"
#include "sensor_calibration.h"
#include "sensor_read.h"

/* Global variable defines */

/* Private variable defines */

/* Private pointers to sensor configurations */
static const MPU6050_Configuration *prv_mpu6050cfg;
static const HMC5983_Configuration *prv_hmc5983cfg;

/* Private pointers to sensor calibrations */
static Sensor_Calibration *prv_accelerometer_cal = NULL;
static Sensor_Calibration *prv_magnetometer_cal = NULL;

/* Private pointer to the Sensor Read Thread */
static Thread *tp = NULL;

/* Temporary holder of sensor data */
static uint8_t temp_data[14];

/* Working area for the sensor read thread */
static WORKING_AREA(waThreadSensorRead, 128);

/* Private function defines */
static void ApplyCalibration(	Sensor_Calibration *cal,
								int16_t raw_data[3], 
								float calibrated_data[3],
								float sensor_gain);
static void MPU6050ConvertAndSave(	MPU6050_Data *dh,
									uint8_t data[14]);
static void HMC5983ConvertAndSave(	HMC5983_Data *dh, 
									uint8_t data[6]);
static msg_t ThreadSensorRead(void *arg);

/* Private external functions */

/**
 * @brief Initializes the sensor read thread and config pointers
 * 
 * @param[in] mpu6050cfg Pointer to MPU6050 config
 * @param[in] hmc5983cfg Pointer to HMC5983 config
 * 
 * @return RDY_OK if the initialization was successful
 */
msg_t SensorReadInit(const MPU6050_Configuration *mpu6050cfg,
					 const HMC5983_Configuration *hmc5983cfg,
					 Sensor_Calibration *accelerometer_cal,
					 Sensor_Calibration *magnetometer_cal)
{
	/* Parameter checks */
	if ((mpu6050cfg == NULL) || (hmc5983cfg == NULL))
		return !RDY_OK; /* Error! */

	prv_mpu6050cfg = mpu6050cfg;
	prv_hmc5983cfg = hmc5983cfg;

	prv_accelerometer_cal = accelerometer_cal;
	prv_magnetometer_cal = magnetometer_cal;

	/* If there are valid calibration pointers, initialize mutexes */
	if (accelerometer_cal != NULL)
		chMtxInit(&accelerometer_cal->lock);

	if (magnetometer_cal != NULL)
		chMtxInit(&magnetometer_cal->lock);

	/* Initialize read thread */
	chThdCreateStatic(	waThreadSensorRead,
						sizeof(waThreadSensorRead), 
						HIGHPRIO, 
						ThreadSensorRead, 
						NULL);

	return RDY_OK;
}

/**
 * @brief MPU6050 external interrupt callback
 * 
 * @param[in] extp 		Pointer to EXT Driver
 * @param[in] channel 	EXT Channel whom fired the interrupt
 */
void MPU6050cb(EXTDriver *extp, expchannel_t channel)
{
	(void)extp;
	(void)channel;

	if (tp != NULL)
	{
		/* Wakes up the sensor read thread */
		chSysLockFromIsr();
		chEvtSignalI(tp, (eventmask_t)MPU6050_DATA_AVAILABLE_EVENTMASK);
		chSysUnlockFromIsr();
	}
}

/**
 * @brief HMC5983 external interrupt callback
 * 
 * @param[in] extp 		Pointer to EXT Driver
 * @param[in] channel 	EXT Channel whom fired the interrupt
 */
void HMC5983cb(EXTDriver *extp, expchannel_t channel)
{
	(void)extp;
	(void)channel;

	if (tp != NULL)
	{
		/* Wakes up the sensor read thread */
		chSysLockFromIsr();
		chEvtSignalI(tp, (eventmask_t)HMC5983_DATA_AVAILABLE_EVENTMASK);
		chSysUnlockFromIsr();
	}
}

/**
 * @brief Converts two bytes in 2's complement form to a signed 16-bit value
 * 
 * @param[in] msb Most significant byte
 * @param[in] lsb Least significant byte
 * 
 * @return Signed 16-bit value
 */
int16_t twoscomplement2signed(uint8_t msb, uint8_t lsb)
{
	return (int16_t)((((uint16_t)msb) << 8) | ((uint16_t)lsb));
}


/* Private functions */

/**
 * @brief Reads data from the sensors whom have new data available
 * 
 * @param[in] arg Unused.
 * @return Never arrives at the return value.
 */
static msg_t ThreadSensorRead(void *arg)
{
	(void)arg;

	eventmask_t events;
	tp = chThdSelf();

	chRegSetThreadName("Sensor Readout");

	while (1)
	{
		/* Waiting for an IRQ to happen.*/
		events = chEvtWaitAny((eventmask_t)(MPU6050_DATA_AVAILABLE_EVENTMASK | 
											HMC5983_DATA_AVAILABLE_EVENTMASK | 
											MS5611_DATA_AVAILABLE_EVENTMASK));

		if (events & MPU6050_DATA_AVAILABLE_EVENTMASK)
		{
			/* Read the data */
			MPU6050ReadData(prv_mpu6050cfg, temp_data);

			/* Lock the data structure while changing it */
			chMtxLock(&prv_mpu6050cfg->data_holder->read_lock);

			/* Convert and save the raw data */
			MPU6050ConvertAndSave(prv_mpu6050cfg->data_holder, temp_data);

			/* Apply calibration and save calibrated data */
			ApplyCalibration(	prv_accelerometer_cal,
								prv_mpu6050cfg->data_holder->raw_accel_data,
								prv_mpu6050cfg->data_holder->accel_data,
								9.81f);

			ApplyCalibration(	NULL,
								prv_mpu6050cfg->data_holder->raw_gyro_data,
								prv_mpu6050cfg->data_holder->gyro_data,
								MPU6050GetGyroGain(prv_mpu6050cfg));

			/* Unlock the data structure */
			chMtxUnlock();



			/* Broadcast new data available */
			chSysLock();
			if (chEvtIsListeningI(&prv_mpu6050cfg->data_holder->es))
				chEvtBroadcastFlagsI(&prv_mpu6050cfg->data_holder->es,
									 (flagsmask_t)MPU6050_DATA_AVAILABLE_EVENTMASK);
			chSysUnlock();
		}

		if (events & HMC5983_DATA_AVAILABLE_EVENTMASK)
		{
			/* Read the data */
			HMC5983ReadData(prv_hmc5983cfg, temp_data);

			/* Lock the data structure while changing it */
			chMtxLock(&prv_hmc5983cfg->data_holder->read_lock);

			/* Convert and save the raw data */
			HMC5983ConvertAndSave(prv_hmc5983cfg->data_holder, temp_data);

			/* Apply calibration and save calibrated data */
			ApplyCalibration(	prv_magnetometer_cal,
								prv_hmc5983cfg->data_holder->raw_mag_data,
								prv_hmc5983cfg->data_holder->mag_data,
								1.0f);

			/* Unlock the data structure */
			chMtxUnlock();

			/* Broadcast new data available */
			chSysLock();
			if (chEvtIsListeningI(&prv_hmc5983cfg->data_holder->es))
				chEvtBroadcastFlagsI(&prv_hmc5983cfg->data_holder->es,
									 (flagsmask_t)HMC5983_DATA_AVAILABLE_EVENTMASK);
			chSysUnlock();
		}

		if (events & MS5611_DATA_AVAILABLE_EVENTMASK)
		{

		}
	}

	return RDY_OK;
}

/**
 * @brief Applies sensor calibration to raw data values.
 * 
 * @details The calibration will provide unity output to the reference
 * 			vector. Use senor_gain to get correct output relative to reality.
 * 
 * @param[in] cal Pointer to calibration structure
 * @param[in] raw_data Pointer to the raw data array
 * @param[out] calibrated_data Pointer to the calibrated data array
 * @param[in] sensor_gain The gain of the sensor after calibration
 */
static void ApplyCalibration(	Sensor_Calibration *cal,
								int16_t raw_data[3], 
								float calibrated_data[3],
								float sensor_gain)
{
	if (cal != NULL)
	{
		chMtxLock(&cal->lock);
		calibrated_data[0] = ((float)raw_data[0] - cal->bias[0]) * cal->gain[0]
							 * sensor_gain;
		calibrated_data[1] = ((float)raw_data[1] - cal->bias[1]) * cal->gain[1]
							 * sensor_gain;
		calibrated_data[2] = ((float)raw_data[2] - cal->bias[2]) * cal->gain[2]
							 * sensor_gain;
		chMtxUnlock();
	}
	else
	{
		calibrated_data[0] = (float)raw_data[0] * sensor_gain;
		calibrated_data[1] = (float)raw_data[1] * sensor_gain;
		calibrated_data[2] = (float)raw_data[2] * sensor_gain;
	}
	
}

/**
 * @brief Converts raw MPU6050 sensor data to signed 16-bit values
 * 
 * @param[out] dh Pointer to data holder structure
 * @param[in] data Pointer to temporary raw data holder
 */
static void MPU6050ConvertAndSave(	MPU6050_Data *dh, 
									uint8_t data[14])
{
	dh->raw_accel_data[0] = twoscomplement2signed(data[0], data[1]);
	dh->raw_accel_data[1] = twoscomplement2signed(data[2], data[3]);
	dh->raw_accel_data[2] = twoscomplement2signed(data[4], data[5]);

	dh->temperature = twoscomplement2signed(data[6], data[7]);

	dh->raw_accel_data[0] = twoscomplement2signed(data[8], data[9]);
	dh->raw_accel_data[1] = twoscomplement2signed(data[10], data[11]);
	dh->raw_accel_data[2] = twoscomplement2signed(data[12], data[13]);
}

/**
 * @brief Converts raw HMC5983 sensor data to signed 16-bit values
 * 
 * @param[out] dh Pointer to data holder structure
 * @param[in] data Pointer to temporary raw data holder
 */
static void HMC5983ConvertAndSave(	HMC5983_Data *dh, 
									uint8_t data[6])
{
	dh->raw_mag_data[0] = twoscomplement2signed(data[0], data[1]);
	dh->raw_mag_data[2] = twoscomplement2signed(data[2], data[3]);
	dh->raw_mag_data[1] = twoscomplement2signed(data[4], data[5]);

}