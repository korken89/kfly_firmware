#ifndef __HMC5983_H
#define __HMC5983_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define HMC5983_ADDRESS                     0x1E

#define HMC5983_RA_CONFIG_A                 0x00
#define HMC5983_RA_CONFIG_B                 0x01
#define HMC5983_RA_MODE                     0x02
#define HMC5983_RA_DATAX_H                  0x03
#define HMC5983_RA_DATAX_L                  0x04
#define HMC5983_RA_DATAZ_H                  0x05
#define HMC5983_RA_DATAZ_L                  0x06
#define HMC5983_RA_DATAY_H                  0x07
#define HMC5983_RA_DATAY_L                  0x08
#define HMC5983_RA_STATUS                   0x09
#define HMC5983_RA_ID_A                     0x0A
#define HMC5983_RA_ID_B                     0x0B
#define HMC5983_RA_ID_C                     0x0C
#define HMC5983_RA_TEMP_H                   0x31
#define HMC5983_RA_TEMP_L                   0x32

#define HMC5983_MS0_BIT                     0
#define HMC5983_MS1_BIT                     1
#define HMC5983_DO0_BIT                     2
#define HMC5983_DO1_BIT                     3
#define HMC5983_DO2_BIT                     4
#define HMC5983_MA0_BIT                     5
#define HMC5983_MA1_BIT                     6
#define HMC5983_CRA7_BIT                    7

#define HMC5983_GN0_BIT                     5
#define HMC5983_GN1_BIT                     6
#define HMC5983_GN2_BIT                     7

#define HMC5983_MD0_BIT                     0
#define HMC5983_MD1_BIT                     1
#define HMC5983_MR2_BIT                     2
#define HMC5983_MR5_BIT                     5
#define HMC5983_MR7_BIT                     7

#define HMC5983_MEAS_MODE_NORMAL            (0 << HMC5983_MS0_BIT)
#define HMC5983_MEAS_MODE_POS_BIAS          (1 << HMC5983_MS0_BIT)
#define HMC5983_MEAS_MODE_NEG_BIAS          (2 << HMC5983_MS0_BIT)
#define HMC5983_MEAS_MODE_TEMPERATURE       (3 << HMC5983_MS0_BIT)

#define HMC5983_DATA_RATE_0D15_HZ           (0 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_1D5_HZ            (1 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_3D0_HZ            (2 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_7D5_HZ            (3 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_15D0_HZ           (4 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_30D0_HZ           (5 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_75D0_HZ           (6 << HMC5983_DO0_BIT)
#define HMC5983_DATA_RATE_220D0_HZ          (7 << HMC5983_DO0_BIT)

#define HMC5983_AVERAGE_1_SAMPLE            (0 << HMC5983_MA0_BIT)
#define HMC5983_AVERAGE_2_SAMPLES           (1 << HMC5983_MA0_BIT)
#define HMC5983_AVERAGE_4_SAMPLES           (2 << HMC5983_MA0_BIT)
#define HMC5983_AVERAGE_8_SAMPLES           (3 << HMC5983_MA0_BIT)

#define HMC5983_TEMPERATURE_ENABLE          (1 << HMC5983_CRA7_BIT)
#define HMC5983_TEMPERATURE_DISABLE         (0 << HMC5983_CRA7_BIT)

#define HMC5983_GAIN_0D88_GA                (0 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_1D3_GA                 (1 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_1D9_GA                 (2 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_2D5_GA                 (3 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_4D0_GA                 (4 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_4D7_GA                 (5 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_5D6_GA                 (6 << HMC5983_GN2_BIT)
#define HMC5983_GAIN_8D1_GA                 (7 << HMC5983_GN2_BIT)

#define HMC5983_OP_MODE_CONTINOUS           (0 << HMC5983_MD0_BIT)
#define HMC5983_OP_MODE_SINGLE              (1 << HMC5983_MD0_BIT)
#define HMC5983_OP_MODE_IDLE                (2 << HMC5983_MD0_BIT)

#define HMC5983_I2C_FAST_ENABLE             (1 << HMC5983_MR7_BIT)
#define HMC5983_I2C_FAST_DISABLE            (0 << HMC5983_MR7_BIT)

#define HMC5983_LOW_POWER_ENABLE            (1 << HMC5983_MR5_BIT)
#define HMC5983_LOW_POWER_DISABLE           (0 << HMC5983_MR5_BIT)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
typedef struct
{
    int16_t raw_mag_data[3];    /* Magnetometer raw data holder         */
    float mag_data[3];          /* Magnetometer calibrated data holder  */
    mutex_t read_lock;          /* Keep listeners from reading if
                                   new data is being written            */
} HMC5983_Data;

typedef struct
{
    uint8_t temperature_sensor;
    uint8_t sample_averaging;
    uint8_t output_rate;
    uint8_t measurement_mode;
    uint8_t gain_configuration;
    uint8_t operating_mode;
    uint8_t i2c_speed;
    uint8_t low_power_mode;
    uint8_t address_7bit;
    HMC5983_Data *data_holder;
    I2CDriver *i2cp;
} HMC5983_Configuration;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
msg_t HMC5983Init(const HMC5983_Configuration *cfg);
msg_t HMC5983GetID(const HMC5983_Configuration *cfg, uint8_t id[3]);
msg_t HMC5983ReadData(const HMC5983_Configuration *cfg, uint8_t data[6]);

#endif
