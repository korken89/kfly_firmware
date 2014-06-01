#include "ch.h"
#include "hal.h"
#include "eicu.h"
#include "myusb.h"
#include "mpu6050.h"
#include "hmc5983.h"
#include "ext_flash.h"
#include "sensor_calibration.h"
#include "sensor_read.h"
#include "rc_output.h"
#include "rc_input.h"
#include "chprintf.h"
#include "serialmanager.h"
#include "estimation.h"
#include "control.h"

volatile const char *kfly_error;

/* I2C interface #2 Configuration */
static const I2CConfig i2cfg2 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

/* SPI1 Configuration */
static const SPIConfig spi1cfg = {
  NULL,
  GPIOA,
  GPIOA_RF_SEL,
  SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL
};

/* External interrupt configuration */
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
        {EXT_CH_MODE_FALLING_EDGE    |
         EXT_CH_MODE_AUTOSTART      |
         EXT_MODE_GPIOC, HMC5983cb},    /* 13: HMC5983 IRQ      */
        {EXT_CH_MODE_RISING_EDGE   |
         EXT_CH_MODE_AUTOSTART      |
         EXT_MODE_GPIOC, MPU6050cb},    /* 14: MPU6050 IRQ      */
        {EXT_CH_MODE_DISABLED, NULL},   /* 15: RF Module IRQ    */
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL}
    }
};

static ExternalFlashData flash_data;
static const ExternalFlashConfig flashcfg = {
    &SPID1,
    GPIOC,
    GPIOC_FLASH_SEL,
    M25PE40_NUM_PAGES,
    FLASH_M25PE40_ID,
    &flash_data
};

int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured 
     *   device drivers and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread
     *   and the RTOS is active.
     */
    halInit();
    chSysInit();

    /*
     *
     * Initializes a serial-over-USB CDC driver.
     * 
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     *
     * Activates the USB driver and then the USB bus pull-up on D+.
     * 
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /*
     *
     * Start the I2C for the sensors
     * 
     */
    i2cStart(&I2CD2, &i2cfg2);

    /*
     *
     * Start the SPI for the external flash and the RF module
     * 
     */
    spiStart(&SPID1, &spi1cfg);

    /*
     *
     * Start the external interrupts
     *
     */
    extStart(&EXTD1, &extcfg);

    /*
     *
     * Start the extended input capture module for RC inputs
     *
     */
    eicuInit();

    /*
     *
     * Initialize the external flash
     *
     */
    if (ExternalFlashInit(&flashcfg) != HAL_SUCCESS)
        chSysHalt("External Flash ID error.");

    /*
     *
     * Initialize RC inputs
     *
     */
    if (RCInputInit(MODE_CPPM_INPUT) != MSG_OK)
        chSysHalt("RC input initialization failed.");

    /*
     *
     * Initialize sensors and read out threads
     *
     */
    if (SensorReadInit() != MSG_OK)
        chSysHalt("Sensor initialization failed.");

    /*
     *
     * Start Serial Manager
     *
     */
    vSerialManagerInit();

    /*
     *
     * Initialize the estimation
     *
     */
    EstimationInit();

    /*
     *
     * Initialize the controllers
     *
     */
    ControlInit();


    while(1)
    {
        palTogglePad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(200);
    }
}
