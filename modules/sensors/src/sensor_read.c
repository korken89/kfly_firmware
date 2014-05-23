/* *
 *
 * Sensor readout handling
 *
 * */

#include "ch.h"
#include "hal.h"
#include "sensor_read.h"

/* Global variable defines */

/* Private variable defines */

/* MPU6050 calibration, data holder and configurations */
static event_source_t new_data_es;
CCM_MEMORY static Sensor_Calibration mpu6050cal;
CCM_MEMORY static MPU6050_Data mpu6050data;
static const MPU6050_Configuration mpu6050cfg = {
    MPU6050_DLPF_BW_42,             /* Digital low-pass filter config     */
    MPU6050_EXT_SYNC_DISABLED,      /* External sync config               */
    MPU6050_GYRO_FS_2000,           /* Gyro range config                  */
    MPU6050_ACCEL_FS_16,            /* Accel range config                 */
    MPU6050_FIFO_DISABLED,          /* FIFO config                        */
    MPU6050_INTMODE_ACTIVEHIGH |
    MPU6050_INTDRIVE_PUSHPULL  |
    MPU6050_INTLATCH_50USPULSE |
    MPU6050_INTCLEAR_ANYREAD,       /* Interrupt config                   */
    MPU6050_INTDRDY_ENABLE,         /* Interrupt enable config            */
    MPU6050_ADDRESS_AD0_HIGH,       /* MPU6050 address                    */
    MPU6050_CLK_X_REFERENCE,        /* Clock reference                    */
    4,                              /* Sample rate divider                */
    &mpu6050data,                   /* Pointer to data holder             */
    &I2CD2                          /* Pointer to I2C Driver              */
};

/* HMC5983 calibration, data holder and configuration */
CCM_MEMORY static Sensor_Calibration hmc5983cal;
CCM_MEMORY static HMC5983_Data hmc5983data;
static const HMC5983_Configuration hmc5983cfg = {
    HMC5983_TEMPERATURE_ENABLE,     /* Enable/disable temperature sensor  */
    HMC5983_AVERAGE_8_SAMPLES,      /* Sample averaging config            */
    HMC5983_DATA_RATE_75D0_HZ,      /* Output data rate config            */
    HMC5983_MEAS_MODE_NORMAL,       /* Measurement mode config            */
    HMC5983_GAIN_1D3_GA,            /* Gain config                        */
    HMC5983_OP_MODE_CONTINOUS,      /* Operating mode config              */
    HMC5983_I2C_FAST_DISABLE,       /* Enable/disable 3.4 MHz I2C         */
    HMC5983_LOW_POWER_DISABLE,      /* Enable/disable low power mode      */
    HMC5983_ADDRESS,                /* HMC5983 address                    */
    &hmc5983data,                   /* Pointer to data holder             */
    &I2CD2                          /* Pointer to I2C Driver              */
};
/* Private pointers to sensor configurations */
static const Sensor_Read_Configuration sensorcfg = {
    &mpu6050cfg,
    &hmc5983cfg,
    &mpu6050cal,
    &hmc5983cal,
    0,
    &new_data_es
};

/* Private pointer to the Sensor Read Thread */
static thread_t *thread_sensor_read_p = NULL;

/* Temporary holder of sensor data */
static uint8_t temp_data[14]; /* NOTE: This variable may NOT be placed in CCM 
                                 memory because DMA directly accesses it. */

/* Working area for the sensor read thread */
CCM_MEMORY static THD_WORKING_AREA(waThreadSensorRead, 128);

/* Private function defines */
static int16_t twoscomplement2signed(uint8_t msb, uint8_t lsb);
static THD_FUNCTION(ThreadSensorRead, arg);
static void ApplyCalibration(Sensor_Calibration *cal,
                             int16_t raw_data[3], 
                             float calibrated_data[3],
                             float sensor_gain);
static void MPU6050ConvertAndSave(MPU6050_Data *dh,
                                  uint8_t data[14]);
static void HMC5983ConvertAndSave(HMC5983_Data *dh, 
                                  uint8_t data[6]);

/* Private external functions */

/**
 * @brief Initializes the sensor read thread and config pointers.
 * 
 * @param[in] mpu6050cfg Pointer to MPU6050 config.
 * @param[in] hmc5983cfg Pointer to HMC5983 config.
 * 
 * @return RDY_OK if the initialization was successful.
 */
msg_t SensorReadInit(void)
{
    /* Parameter checks */
    if ((sensorcfg.mpu6050cfg == NULL) || (sensorcfg.hmc5983cfg == NULL))
        return MSG_RESET; /* Error! */

    /* Initialize Accelerometer and Gyroscope */
    if (MPU6050Init(sensorcfg.mpu6050cfg) != MSG_OK)
        return MSG_RESET; /* Initialization failed */

    /* Initialize Magnetometer */
    if (HMC5983Init(sensorcfg.hmc5983cfg) != MSG_OK)
        return MSG_RESET; /* Initialization failed */

    /* Initialize Barometer */
    /* TODO: Add barometer code */

    /* If there are valid calibration pointers, initialize mutexes */
    if (sensorcfg.mpu6050cal != NULL)
        chMtxObjectInit(&sensorcfg.mpu6050cal->lock);

    if (sensorcfg.hmc5983cal != NULL)
        chMtxObjectInit(&sensorcfg.hmc5983cal->lock);

    /* Initialize new data event source */
    osalEventObjectInit(sensorcfg.new_data_es);

    /* Initialize calibration */
    mpu6050cal.gain[0] = mpu6050cal.gain[1] = mpu6050cal.gain[2] = 1.0f;
    hmc5983cal.gain[0] = hmc5983cal.gain[1] = hmc5983cal.gain[2] = 1.0f;
    mpu6050cal.bias[0] = mpu6050cal.bias[1] = mpu6050cal.bias[2] = 0.0f;
    hmc5983cal.bias[0] = hmc5983cal.bias[1] = hmc5983cal.bias[2] = 0.0f;

    /* Initialize read thread */
    chThdCreateStatic(waThreadSensorRead,
                      sizeof(waThreadSensorRead), 
                      HIGHPRIO, 
                      ThreadSensorRead, 
                      NULL);

    return MSG_OK;
}

/**
 * @brief Updates a sensor caibration with new values.
 * 
 * @param[in] cal   Pointer to calibration structure.
 * @param[in] bias  Pointer to new bias (3 values vector).
 * @param[in] gain  Pointer to new gain (3 values vector).
 */
void SetSensorCalibration(Sensor_Calibration *cal, float bias[3], float gain[3])
{
    int i;

    chMtxLock(&cal->lock);

    for (i = 0; i < 3; i++)
    {
        cal->bias[i] = bias[i];
        cal->gain[i] = gain[i];    
    }
    
    chMtxUnlock(&cal->lock);
}

/**
 * @brief MPU6050 external interrupt callback.
 * 
 * @param[in] extp      Pointer to EXT Driver.
 * @param[in] channel   EXT Channel whom fired the interrupt.
 */
void MPU6050cb(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    if (thread_sensor_read_p != NULL)
    {
        /* Wakes up the sensor read thread */
        chSysLockFromISR();
        chEvtSignalI(thread_sensor_read_p, ACCGYRO_DATA_AVAILABLE_EVENTMASK);
        chSysUnlockFromISR();
    }
}

/**
 * @brief HMC5983 external interrupt callback.
 * 
 * @param[in] extp      Pointer to EXT Driver.
 * @param[in] channel   EXT Channel whom fired the interrupt.
 */
void HMC5983cb(EXTDriver *extp, expchannel_t channel)
{
    (void)extp;
    (void)channel;

    if (thread_sensor_read_p != NULL)
    {
        /* Wakes up the sensor read thread */
        chSysLockFromISR();
        chEvtSignalI(thread_sensor_read_p, MAG_DATA_AVAILABLE_EVENTMASK);
        chSysUnlockFromISR();
    }
}

/**
 * @brief       Get the new data event source.
 * 
 * @return      Pointer to the event source.
 *          
 */
event_source_t *ptrGetNewDataEventSource(void)
{
    return sensorcfg.new_data_es;
}

/**
 * @brief   Get the raw accelerometer data.
 * 
 * @return  Pointer to the raw accelerometer data.
 * 
 * @note    This is an unsafe function as it does not lock the sensor
 *          structures. This can cause erroneous data to be read. A call to
 *          LockSensorStructuresForRead and UnlockSensorStructuresForRead is 
 *          recommended while copying the data.
 */
int16_t *ptrGetRawAccelerometerData(void)
{
    return sensorcfg.mpu6050cfg->data_holder->raw_accel_data;
}

/**
 * @brief   Get the accelerometer data.
 * 
 * @return  Pointer to the accelerometer data.
 * 
 * @note    This is an unsafe function as it does not lock the sensor
 *          structures. This can cause erroneous data to be read. A call to
 *          LockSensorStructuresForRead and UnlockSensorStructuresForRead is 
 *          recommended while copying the data.
 */
float *ptrGetAccelerometerData(void)
{
    return sensorcfg.mpu6050cfg->data_holder->accel_data;
}

/**
 * @brief   Get the raw gyro data.
 * 
 * @return  Pointer to the raw gyro data.
 * 
 * @note    This is an unsafe function as it does not lock the sensor
 *          structures. This can cause erroneous data to be read. A call to
 *          LockSensorStructuresForRead and UnlockSensorStructuresForRead is 
 *          recommended while copying the data.
 */
int16_t *ptrGetRawGyroscopeData(void)
{
    return sensorcfg.mpu6050cfg->data_holder->raw_gyro_data;
}

/**
 * @brief   Get the gyro data.
 * 
 * @return  Pointer to the gyro data.
 * 
 * @note    This is an unsafe function as it does not lock the sensor
 *          structures. This can cause erroneous data to be read. A call to
 *          LockSensorStructuresForRead and UnlockSensorStructuresForRead is 
 *          recommended while copying the data.
 */
float *ptrGetGyroscopeData(void)
{
    return sensorcfg.mpu6050cfg->data_holder->gyro_data;
}

/**
 * @brief   Get the raw gyro temperature data.
 * 
 * @return  Pointer to the raw gyro temperature data.
 */
int16_t GetRawGyroscopeTemperature(void)
{
    return sensorcfg.mpu6050cfg->data_holder->raw_temperature;
}

/**
 * @brief   Get the gyro temperature data.
 * 
 * @return  Pointer to the gyro temperature data.
 */
float GetGyroscopeTemperature(void)
{
    return sensorcfg.mpu6050cfg->data_holder->temperature;
}

/**
 * @brief   Get the raw magnetometer data.
 * 
 * @return  Pointer to the raw magnetometer data.
 * 
 * @note    This is an unsafe function as it does not lock the sensor
 *          structures. This can cause erroneous data to be read. A call to
 *          LockSensorStructuresForRead and UnlockSensorStructuresForRead is 
 *          recommended while copying the data.
 */
int16_t *ptrGetRawMagnetometerData(void)
{
    return sensorcfg.hmc5983cfg->data_holder->raw_mag_data;
}

/**
 * @brief   Get the magnetometer data.
 * 
 * @return  Pointer to the magnetometer data.
 * 
 * @note    This is an unsafe function as it does not lock the sensor
 *          structures. This can cause erroneous data to be read. A call to
 *          LockSensorStructuresForRead and UnlockSensorStructuresForRead is 
 *          recommended while copying the data.
 */
float *ptrGetMagnetometerData(void)
{
    return sensorcfg.hmc5983cfg->data_holder->mag_data;
}

/**
 * @brief       Get all the IMU data.
 * @param[out]  data  Pointer to IMU_Data structure in which to save the data.
 */
void GetIMUData(IMU_Data *data)
{
    int i;

    /* Lock data structures before reading */
    LockSensorStructures();

    /* Copy data to the requested IMU structure */
    for (i = 0; i < 3; i++)
    {
        data->accelerometer[i] = 
                        sensorcfg.mpu6050cfg->data_holder->accel_data[i];
        data->gyroscope[i] = 
                        sensorcfg.mpu6050cfg->data_holder->gyro_data[i];
        data->magnetometer[i] = 
                        sensorcfg.hmc5983cfg->data_holder->mag_data[i];
    }
    
    data->temperature = sensorcfg.mpu6050cfg->data_holder->temperature;

    /* Unlock data structures after reading */
    UnlockSensorStructures();
}

/**
 * @brief       Get all the raw IMU data.
 * @param[out]  data  Pointer to IMU_RawData structure in which to
 *                    save the data.
 */
void GetRawIMUData(IMU_RawData *data)
{
    int i;

    /* Lock data structures before reading */
    LockSensorStructures();

    /* Copy data to the requested IMU structure */
    for (i = 0; i < 3; i++)
    {
        data->accelerometer[i] =
                        sensorcfg.mpu6050cfg->data_holder->raw_accel_data[i];
        data->gyroscope[i] = 
                        sensorcfg.mpu6050cfg->data_holder->raw_gyro_data[i];
        data->magnetometer[i] = 
                        sensorcfg.hmc5983cfg->data_holder->raw_mag_data[i];
    }
    
    data->temperature = sensorcfg.mpu6050cfg->data_holder->raw_temperature;

    /* Unlock data structures after reading */
    UnlockSensorStructures();
}

/**
 * @brief       Get the IMU calibration data.
 * @param[out]  data  Pointer to IMU_Calibration structure in which to
 *                    save the data.
 */
void GetIMUCalibration(IMU_Calibration *cal)
{
    int i;

    /* Lock calibration structures before reading */
    LockSensorCalibration();

    /* Copy data to the requested IMU structure */
    for (i = 0; i < 3; i++)
    {
        cal->accelerometer_bias[i] = sensorcfg.mpu6050cal->bias[i];
        cal->accelerometer_gain[i] = sensorcfg.mpu6050cal->gain[i];
        cal->magnetometer_bias[i]  = sensorcfg.hmc5983cal->bias[i];
        cal->magnetometer_gain[i]  = sensorcfg.hmc5983cal->gain[i];
    }
    
    cal->timestamp = sensorcfg.calibration_timestamp;

    /* Unlock calibration structures after reading */
    UnlockSensorCalibration();
}

/**
 * @brief Lock the sensor data registers to safely read them.
 * @note  An UnlockSensorStructures must be called as soon as read is
 *        finished because this prohibits the sensor read thread to access the
 *        sensor registers for saving data. hence it shall be locked as short
 *        a time as possible while copying the data.
 */
void LockSensorStructures(void)
{
    chMtxLock(&sensorcfg.mpu6050cfg->data_holder->read_lock);
    chMtxLock(&sensorcfg.hmc5983cfg->data_holder->read_lock);
}

/**
 * @brief Unlock the sensor data registers.
 */
void UnlockSensorStructures(void)
{
    chMtxUnlock(&sensorcfg.hmc5983cfg->data_holder->read_lock);
    chMtxUnlock(&sensorcfg.mpu6050cfg->data_holder->read_lock);
}


/**
 * @brief Lock the sensor calibration registers to safely read and write to
 *        them.
 * @note  An UnlockSensorCalibration must be called as soon as read is
 *        finished because this prohibits the sensor read thread to access the
 *        sensor registers for saving data. hence it shall be locked as short
 *        a time as possible while copying the data.
 */
void LockSensorCalibration(void)
{
    chMtxLock(&sensorcfg.mpu6050cal->lock);
    chMtxLock(&sensorcfg.hmc5983cal->lock);
}

/**
 * @brief Unlock the sensor calibration registers.
 */
void UnlockSensorCalibration(void)
{
    chMtxUnlock(&sensorcfg.hmc5983cal->lock);
    chMtxUnlock(&sensorcfg.mpu6050cal->lock);
}


/**
 * @brief   Get the accelerometer calibration.
 * 
 * @return  Pointer to the accelerometer calibration.
 */
Sensor_Calibration *ptrGetAccelerometerCalibration(void)
{
    return &mpu6050cal;
}

/**
 * @brief   Get the magnetometer calibration.
 * 
 * @return  Pointer to the magnetometer calibration.
 */
Sensor_Calibration *ptrGetMagnetometerCalibration(void)
{
    return &hmc5983cal;
}

/**
 * @brief Converts two bytes in 2's complement form to a signed 16-bit value.
 * 
 * @param[in] msb Most significant byte.
 * @param[in] lsb Least significant byte.
 * 
 * @return Signed 16-bit value.
 */
static int16_t twoscomplement2signed(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((((uint16_t)msb) << 8) | ((uint16_t)lsb));
}


/* Private functions */

/**
 * @brief Reads data from the sensors whom have new data available.
 * 
 * @param[in] arg Unused.
 * @return Never arrives at the return value.
 */
static THD_FUNCTION(ThreadSensorRead, arg)
{
    (void)arg;

    eventmask_t events;
    thread_sensor_read_p = chThdGetSelfX();

    chRegSetThreadName("Sensor Readout");

    while (1)
    {
        /* Waiting for an IRQ to happen.*/
        events = chEvtWaitAny((eventmask_t)(ACCGYRO_DATA_AVAILABLE_EVENTMASK | 
                                            MAG_DATA_AVAILABLE_EVENTMASK | 
                                            BARO_DATA_AVAILABLE_EVENTMASK));

        if (events & ACCGYRO_DATA_AVAILABLE_EVENTMASK)
        {
            /* Read the data */
            MPU6050ReadData(sensorcfg.mpu6050cfg, temp_data);

            /* Lock the data structure while changing it */
            chMtxLock(&sensorcfg.mpu6050cfg->data_holder->read_lock);

            /* Convert and save the raw data */
            MPU6050ConvertAndSave(sensorcfg.mpu6050cfg->data_holder, temp_data);

            /* Apply calibration and save calibrated data */
            ApplyCalibration(sensorcfg.mpu6050cal,
                             sensorcfg.mpu6050cfg->data_holder->raw_accel_data,
                             sensorcfg.mpu6050cfg->data_holder->accel_data,
                             1.0f);

            ApplyCalibration(NULL,
                             sensorcfg.mpu6050cfg->data_holder->raw_gyro_data,
                             sensorcfg.mpu6050cfg->data_holder->gyro_data,
                             MPU6050GetGyroGain(sensorcfg.mpu6050cfg));

            /* Unlock the data structure */
            chMtxUnlock(&sensorcfg.mpu6050cfg->data_holder->read_lock);

            /* Broadcast new data available */
            osalSysLock();

            if (chEvtIsListeningI(sensorcfg.new_data_es))
                chEvtBroadcastFlagsI(sensorcfg.new_data_es,
                                     ACCGYRO_DATA_AVAILABLE_EVENTMASK);

            osalSysUnlock();
        }

        if (events & MAG_DATA_AVAILABLE_EVENTMASK)
        {
            /* Read the data */
            HMC5983ReadData(sensorcfg.hmc5983cfg, temp_data);

            /* Lock the data structure while changing it */
            chMtxLock(&sensorcfg.hmc5983cfg->data_holder->read_lock);

            /* Convert and save the raw data */
            HMC5983ConvertAndSave(sensorcfg.hmc5983cfg->data_holder, temp_data);

            /* Apply calibration and save calibrated data */
            ApplyCalibration(NULL,
                             sensorcfg.hmc5983cfg->data_holder->raw_mag_data,
                             sensorcfg.hmc5983cfg->data_holder->mag_data,
                             1.0f);

            /* Unlock the data structure */
            chMtxUnlock(&sensorcfg.hmc5983cfg->data_holder->read_lock);

            /* Broadcast new data available */
            osalSysLock();

            if (chEvtIsListeningI(sensorcfg.new_data_es))
                chEvtBroadcastFlagsI(sensorcfg.new_data_es,
                                     MAG_DATA_AVAILABLE_EVENTMASK);

            osalSysUnlock();
        }

        if (events & BARO_DATA_AVAILABLE_EVENTMASK)
        {

        }
    }

    return MSG_OK;
}

/**
 * @brief Applies sensor calibration to raw data values.
 * 
 * @details The calibration will provide unity output to the reference
 *          vector. Use senor_gain to get correct output relative to reality.
 * 
 * @param[in] cal Pointer to calibration structure.
 * @param[in] raw_data Pointer to the raw data array.
 * @param[out] calibrated_data Pointer to the calibrated data array.
 * @param[in] sensor_gain The gain of the sensor after calibration.
 */
static void ApplyCalibration(Sensor_Calibration *cal,
                             int16_t raw_data[3], 
                             float calibrated_data[3],
                             float sensor_gain)
{
    if (cal != NULL)
    {
        chMtxLock(&cal->lock);
        calibrated_data[0] = (((float)raw_data[0]) - cal->bias[0]) * cal->gain[0]
                             * sensor_gain;
        calibrated_data[1] = (((float)raw_data[1]) - cal->bias[1]) * cal->gain[1]
                             * sensor_gain;
        calibrated_data[2] = (((float)raw_data[2]) - cal->bias[2]) * cal->gain[2]
                             * sensor_gain;
        chMtxUnlock(&cal->lock);
    }
    else
    {
        calibrated_data[0] = ((float)raw_data[0]) * sensor_gain;
        calibrated_data[1] = ((float)raw_data[1]) * sensor_gain;
        calibrated_data[2] = ((float)raw_data[2]) * sensor_gain;
    }
    
}

/**
 * @brief Converts raw MPU6050 sensor data to signed 16-bit values.
 * 
 * @param[out] dh Pointer to data holder structure.
 * @param[in] data Pointer to temporary raw data holder.
 */
static void MPU6050ConvertAndSave(MPU6050_Data *dh, uint8_t data[14])
{
    /*
     * The accelerometer and gyro is rotated on the board and conversion
     * is needed as for both sensors:
     * - Board XYZ = Sensor XYZ -
     *           x =  y
     *           y = -x
     *           z =  z
     */

    dh->raw_accel_data[0] = twoscomplement2signed(data[2], data[3]);
    dh->raw_accel_data[1] = -twoscomplement2signed(data[0], data[1]);
    dh->raw_accel_data[2] = twoscomplement2signed(data[4], data[5]);

    dh->temperature = twoscomplement2signed(data[6], data[7]);

    dh->raw_gyro_data[0] = twoscomplement2signed(data[10], data[11]);
    dh->raw_gyro_data[1] = -twoscomplement2signed(data[8], data[9]);
    dh->raw_gyro_data[2] = twoscomplement2signed(data[12], data[13]);
}

/**
 * @brief Converts raw HMC5983 sensor data to signed 16-bit values.
 * 
 * @param[out] dh Pointer to data holder structure.
 * @param[in] data Pointer to temporary raw data holder.
 */
static void HMC5983ConvertAndSave(HMC5983_Data *dh, uint8_t data[6])
{
    /*
     * The magnetometer is rotated on the board and conversion is needed as:
     * - Board XYZ = Sensor XYZ -
     *           x = -y
     *           y =  x
     *           z = -z
     */
    dh->raw_mag_data[0] = -twoscomplement2signed(data[4], data[5]);
    dh->raw_mag_data[1] = twoscomplement2signed(data[0], data[1]);
    dh->raw_mag_data[2] = -twoscomplement2signed(data[2], data[3]);
}
