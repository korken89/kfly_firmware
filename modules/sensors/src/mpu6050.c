/* *
 *
 * Abstraction Layer for MPU6050 Accelerometer & Gyroscope
 *
 * */

#include "ch.h"
#include "hal.h"
#include "mpu6050.h"

/* Global variable defines */

/* Private function defines */

/* Private external functions */

/**
 * @brief Initializes the MPU6050 sensor
 * 
 * @param[in] cfg Pointer to configuration structure
 * @return RDY_OK if the initialization was successful
 */
msg_t MPU6050Init(const MPU6050_Configuration *cfg)
{
    static uint8_t txbuf[2];
    msg_t status = MSG_OK;


    /* Error: Pointers not defined */
    if ((cfg->data_holder == NULL) || (cfg->i2cp == NULL))
        return MSG_RESET;


    /* Initialize the data event source and mutex */
    chMtxObjectInit(&cfg->data_holder->read_lock);
    
    /* Perform sensor reset */
    status = MPU6050DeviceReset(cfg);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Initialize the sensor */
    /* Set internal clock source and Sleep mode */
    txbuf[0] = MPU6050_RA_PWR_MGMT_1;   /* Power management register 1 */
    txbuf[1] = cfg->clock_reference;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set FIFO */
    txbuf[0] = MPU6050_RA_FIFO_EN;      /* FIFO Enable register */
    txbuf[1] = cfg->fifo_cfg;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set Gyro range */
    txbuf[0] = MPU6050_RA_GYRO_CONFIG;  /* Gyro configuration register */
    txbuf[1] = cfg->gyro_range_sel;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set Accelerometer range */
    txbuf[0] = MPU6050_RA_ACCEL_CONFIG; /* Accel configuration register */
    txbuf[1] = cfg->accel_range_sel;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set Digital Low-Pass Filter and External Sync */
    txbuf[0] = MPU6050_RA_CONFIG;       /* Configuration register */
    txbuf[1] = (cfg->dlp_cfg | cfg->ext_sync_cfg);
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set Sample Rate divider */
    txbuf[0] = MPU6050_RA_SMPLRT_DIV;   /* Sample rate register */
    txbuf[1] = cfg->sample_rate_divider;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set interrupt pin config */
    txbuf[0] = MPU6050_RA_INT_PIN_CFG;  /* Interrupt pin register */
    txbuf[1] = cfg->int_pin_cfg;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Set interrupts */
    txbuf[0] = MPU6050_RA_INT_ENABLE;   /* Interrupt enable register */
    txbuf[1] = cfg->int_cfg;
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    return status;
}

/**
 * @brief Resets the MPU6050 sensor and the internal signal paths
 * 
 * @param[in] cfg Pointer to configuration structure
 * @return RDY_OK if the initialization was successful
 */
msg_t MPU6050DeviceReset(const MPU6050_Configuration *cfg)
{
    static uint8_t txbuf[2] = {MPU6050_RA_PWR_MGMT_1, MPU6050_DEVICE_RESET};
    msg_t status = MSG_OK;

    /* Reset device */
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Sleep for 100 ms as per datasheet */
    chThdSleepMilliseconds(100);

    /* Reset signal paths */
    txbuf[0] = MPU6050_RA_SIGNAL_PATH_RESET;
    txbuf[1] = (MPU6050_GYRO_RESET | MPU6050_ACCEL_RESET | MPU6050_TEMP_RESET);
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp,
                                        cfg->address_7bit, 
                                        txbuf, 
                                        2, 
                                        NULL, 
                                        0, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Error check */
    if (status != MSG_OK)
        return status;

    /* Sleep for 100 ms as per datasheet */
    chThdSleepMilliseconds(100);

    return status;
}

/**
 * @brief Reads the ID of the MPU6050 sensor
 * 
 * @param[in] cfg Pointer to configuration structure
 * @param[out] id Pointer to where the single byte id will be saved
 *  
 * @return RDY_OK if the initialization was successful
 */
msg_t MPU6050GetID(const MPU6050_Configuration *cfg, uint8_t id[1])
{
    static uint8_t txbuf[1] = {MPU6050_RA_WHO_AM_I};
    msg_t status = MSG_OK;

    /* Get ID */
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp, 
                                        cfg->address_7bit, 
                                        txbuf, 
                                        1, 
                                        id, 
                                        1, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    /* Get the six middle bits */
    *id &= ~0x81;

    return status;
}

/**
 * @brief Reads the data registers of the MPU6050
 *
 * @param[in] cfg Pointer to configuration structure
 * @param[out] data Pointer to where the data will be saved
 * 
 * @return [description]
 */
msg_t MPU6050ReadData(const MPU6050_Configuration *cfg, uint8_t data[14])
{
    static uint8_t txbuf[1] = {MPU6050_RA_ACCEL_XOUT_H};
    msg_t status = MSG_OK;

    /* Get ID */
    i2cAcquireBus(cfg->i2cp);
    status = i2cMasterTransmitTimeout(  cfg->i2cp, 
                                        cfg->address_7bit, 
                                        txbuf, 
                                        1, 
                                        data, 
                                        14, 
                                        OSAL_MS2ST(20));
    i2cReleaseBus(cfg->i2cp);

    return status;
}

/**
 * @brief Get the gain of the accelerometer
 * 
 * @param[in] cfg Pointer to configuration structure
 * @return The gain of the accelerometer
 */
float MPU6050GetAccelGain(const MPU6050_Configuration *cfg)
{
    if (cfg->gyro_range_sel == MPU6050_ACCEL_FS_2)
        return MPU6050_LSB_TO_2G;

    else if (cfg->gyro_range_sel == MPU6050_ACCEL_FS_4)
        return MPU6050_LSB_TO_4G;

    else if (cfg->gyro_range_sel == MPU6050_ACCEL_FS_8)
        return MPU6050_LSB_TO_8G;

    else if (cfg->gyro_range_sel == MPU6050_ACCEL_FS_16)
        return MPU6050_LSB_TO_16G;

    else
        return 0.0f;
}

/**
 * @brief Get the gain of the gyroscope
 * 
 * @param[in] cfg Pointer to configuration structure
 * @return The gain of the gyroscope
 */
float MPU6050GetGyroGain(const MPU6050_Configuration *cfg)
{
    if (cfg->gyro_range_sel == MPU6050_GYRO_FS_250)
        return MPU6050_DPS250_TO_RADPS;

    else if (cfg->gyro_range_sel == MPU6050_GYRO_FS_500)
        return MPU6050_DPS500_TO_RADPS;

    else if (cfg->gyro_range_sel == MPU6050_GYRO_FS_1000)
        return MPU6050_DPS1000_TO_RADPS;

    else if (cfg->gyro_range_sel == MPU6050_GYRO_FS_2000)
        return MPU6050_DPS2000_TO_RADPS;

    else
        return 0.0f;
}

/**
 * @brief Get the sample time of the sensor
 * 
 * @param[in] cfg Pointer to configuration structure
 * @return The sample time of the sensor in seconds
 */
float MPU6050GetSampleTime(const MPU6050_Configuration *cfg)
{
    float fs, rate_div;

    /* MPU6050 internal sample rate is 8 kHz if DLP is off, else 1 kHz. */
    if (cfg->dlp_cfg == MPU6050_DLPF_BW_256)
        fs = 8000.0f;
    else
        fs = 1000.0f;

    rate_div = (float)(cfg->sample_rate_divider + 1);

    return (rate_div / fs);
}
