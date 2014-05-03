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
#include "sensor_read.h"
#include "sensor_calibration.h"

/* Global variable defines */

/* Private variable defines */
static const EXTConfig extcfg = {
	{
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_RISING_EDGE    |
		 EXT_CH_MODE_AUTOSTART 		|
		 EXT_MODE_GPIOC, HMC5983cb}, /* 13: HMC5983 IRQ */
		{EXT_CH_MODE_FALLING_EDGE   |
		 EXT_CH_MODE_AUTOSTART 		|
		 EXT_MODE_GPIOC, MPU6050cb}, /* 14: MPU6050 IRQ */
		{EXT_CH_MODE_FALLING_EDGE   |
		 EXT_CH_MODE_AUTOSTART 		|
		 EXT_MODE_GPIOC, NULL}, /* 15: RF Module IRQ */
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL},
		{EXT_CH_MODE_DISABLED, NULL}
	}
};

/* Private pointers to sensor configurations */
static const MPU6050_Configuration *prv_mpu6050cfg;
static const HMC5983_Configuration *prv_hmc5983cfg;

/* Private pointers to sensor calibrations */
static Sensor_Calibration *prv_mpu6050cal = NULL;
static Sensor_Calibration *prv_hmc5983cal = NULL;

/* Private pointer to the Sensor Read Thread */
static Thread *tp = NULL;

/* Temporary holder of sensor data */
static uint8_t temp_data[14];

/* Private function defines */
static void ApplyCalibration(	Sensor_Calibration *cal,
								int16_t raw_data[3], 
								float calibrated_data[3],
								float sensor_gain);
static void MPU6050ConvertAndSave(	MPU6050_Data *dh,
									uint8_t data[14]);
static void HMC5983ConvertAndSave(	HMC5983_Data *dh, 
									uint8_t data[6]);

/* Private external functions */

/*
 * Sensor readout thread
 */
static WORKING_AREA(waThreadSensorRead, 128);
static msg_t ThreadSensorRead(void *arg)
{
	(void)arg;

	eventmask_t events;
	tp = chThdSelf();

	chRegSetThreadName("Sensor Readout");

	while (1)
	{
		/* Waiting for an IRQ to happen.*/
		events = chEvtWaitAny((eventmask_t)(MPU6050_DATA_AVAILABLE_MASK | 
											HMC5983_DATA_AVAILABLE_MASK | 
											MS5611_DATA_AVAILABLE_MASK));

		if (events & MPU6050_DATA_AVAILABLE_MASK)
		{
			/* Read the data */
			MPU6050ReadData(prv_mpu6050cfg, temp_data);

			/* Lock the data structure while changing it */
			chMtxLock(&prv_mpu6050cfg->data_holder->read_lock);

			/* Convert and save the raw data */
			MPU6050ConvertAndSave(prv_mpu6050cfg->data_holder, temp_data);

			/* Apply calibration and save calibrated data */
			ApplyCalibration(	prv_mpu6050cal,
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
			chEvtBroadcastFlags(&prv_mpu6050cfg->data_holder->es,
								(flagsmask_t)MPU6050_DATA_AVAILABLE_MASK);
		}

		if (events & HMC5983_DATA_AVAILABLE_MASK)
		{
			/* Read the data */
			HMC5983ReadData(prv_hmc5983cfg, temp_data);

			/* Lock the data structure while changing it */
			chMtxLock(&prv_hmc5983cfg->data_holder->read_lock);

			/* Convert and save the raw data */
			HMC5983ConvertAndSave(prv_hmc5983cfg->data_holder, temp_data);

			/* Apply calibration and save calibrated data */
			ApplyCalibration(	prv_hmc5983cal,
								prv_hmc5983cfg->data_holder->raw_mag_data,
								prv_hmc5983cfg->data_holder->mag_data,
								1.0f);

			/* Unlock the data structure */
			chMtxUnlock();

			/* Broadcast new data available */
			chEvtBroadcastFlags(&prv_hmc5983cfg->data_holder->es,
								(flagsmask_t)HMC5983_DATA_AVAILABLE_MASK);
		}

		if (events & MS5611_DATA_AVAILABLE_MASK)
		{

		}
	}

	return RDY_OK;
}

msg_t SensorReadInit(const MPU6050_Configuration *mpu6050cfg,
					 const HMC5983_Configuration *hmc5983cfg)
{
	prv_mpu6050cfg = mpu6050cfg;
	prv_hmc5983cfg = hmc5983cfg;

	/* Initialize read thread */
	chThdCreateStatic(	waThreadSensorRead,
						sizeof(waThreadSensorRead), 
						HIGHPRIO, 
						ThreadSensorRead, 
						NULL);

	return RDY_OK;
}

static void ApplyCalibration(	Sensor_Calibration *cal,
								int16_t raw_data[3], 
								float calibrated_data[3],
								float sensor_gain)
{
	if (cal != NULL)
	{
		calibrated_data[0] = ((float)raw_data[0] - cal->bias[0]) * cal->gain[0]
							 * sensor_gain;
		calibrated_data[1] = ((float)raw_data[1] - cal->bias[1]) * cal->gain[1]
							 * sensor_gain;
		calibrated_data[2] = ((float)raw_data[2] - cal->bias[2]) * cal->gain[2]
							 * sensor_gain;
	}
	else
	{
		calibrated_data[0] = (float)raw_data[0] * sensor_gain;
		calibrated_data[1] = (float)raw_data[1] * sensor_gain;
		calibrated_data[2] = (float)raw_data[2] * sensor_gain;
	}
	
}

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

static void HMC5983ConvertAndSave(	HMC5983_Data *dh, 
									uint8_t data[6])
{
	dh->raw_mag_data[0] = twoscomplement2signed(data[0], data[1]);
	dh->raw_mag_data[2] = twoscomplement2signed(data[2], data[3]);
	dh->raw_mag_data[1] = twoscomplement2signed(data[4], data[5]);

}

void MPU6050cb(EXTDriver *extp, expchannel_t channel)
{
	(void)extp;
	(void)channel;

	if (tp != NULL)
	{
		/* Wakes up the sensor read thread */
		chSysLockFromIsr();
		chEvtSignalI(tp, (eventmask_t)MPU6050_DATA_AVAILABLE_MASK);
		chSysUnlockFromIsr();
	}
}

void HMC5983cb(EXTDriver *extp, expchannel_t channel)
{
	(void)extp;
	(void)channel;

	if (tp != NULL)
	{
		/* Wakes up the sensor read thread */
		chSysLockFromIsr();
		chEvtSignalI(tp, (eventmask_t)HMC5983_DATA_AVAILABLE_MASK);
		chSysUnlockFromIsr();
	}
}

int16_t twoscomplement2signed(uint8_t msb, uint8_t lsb)
{
	return (int16_t)((((uint16_t)msb) << 8) | ((uint16_t)lsb));
}