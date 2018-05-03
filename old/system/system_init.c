/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "system_init.h"

/* All includes from modules */
#include "eicu.h"
#include "usb_access.h"
#include "mpu6050.h"
#include "hmc5983.h"
#include "flash_save.h"
#include "sensor_calibration.h"
#include "sensor_read.h"
#include "rc_output.h"
#include "rc_input.h"
#include "chprintf.h"
#include "serialmanager.h"
#include "estimation.h"
#include "control.h"
#include "motion_capture.h"
#include "system_information.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief Bool holding the state of the shutdown request.
 */
static volatile bool shutdown_requested = false;

/**
 * @brief Holder for the base pointer for the critical tasks linked list.
 */
static system_critical_subscription_t *base_subscription = NULL;


static volatile system_state_t system_state = SYSTEM_UNINITIALIZED;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*
 * @brief   The function containing all drivers and modules to be initialized.
 *          All initialization should be here (except internal inits).
 *
 * @note    ChibiOS Sys init and HAL init shall not be in here, it's in main.
 */
static void vSystemInitList(void)
{
    /*
     *
     * Add all driver and module initializations here!
     *
     */

    /* I2C interface 2 Configuration:
     *
     * Speed 400 kHz, I2C fast mode Tlow/Thigh = 2
     */
    static const I2CConfig i2cfg2 = {
        OPMODE_I2C,
        400000,
        FAST_DUTY_CYCLE_2,
    };

    /* SPI1 Configuration:
     *
     * Speed 5.25MHz, CPHA = 1, CPOL = 1, 8-bit frames, MSB transmitted first.
     * The slave select line is the pin GPIOA_RF_SEL on the port GPIOA.
     */
    static const SPIConfig spi1cfg = {
      NULL,
      GPIOA,
      GPIOA_RF_SEL,
      SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL
    };

    /* External interrupt configuration:
     *
     * HMC5983 IRQ: Falling edge, on GPIOC, auto start
     * MPU6050 IRQ: Rising edge, on GPIOC, auto start
     * RF Module IRQ: Not implemented
     */
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
            {EXT_CH_MODE_FALLING_EDGE   |
             EXT_CH_MODE_AUTOSTART      |
             EXT_MODE_GPIOC, HMC5983cb},    /* 13: HMC5983 IRQ      */
            {EXT_CH_MODE_RISING_EDGE    |
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


    /*
     *
     * Initializes the serial-over-USB CDC driver.
     *
     */
    USBInit();

    /* Activates the USB driver and then the USB bus pull-up on D+. */
    USBDisconnect();
    chThdSleepMilliseconds(500);
    USBConnect();

    /*
     *
     * Start the I2C for the sensors.
     *
     */
    i2cStart(&I2CD2, &i2cfg2);

    /*
     *
     * Start the SPI for the external flash and the RF module.
     *
     */
    spiStart(&SPID1, &spi1cfg);

    /*
     *
     * Start the external interrupts.
     *
     */
    extStart(&EXTD1, &extcfg);

    /*
     *
     * Start the extended input capture module for RC inputs.
     *
     */
    eicuInit();

    /*
     *
     * Initialize the external flash and save to flash functionality.
     * Note: Must be initialized before any module that uses the save to or
     *       read from flash functionality.
     *
     */
    FlashSaveInit();

    /*
     *
     * Initialize the RC inputs.
     *
     */
    RCInputInit();

    /*
     *
     * Initialize the RC Outputs
     *
     */
    RCOutputInit();

    /*
     *
     * Initialize the sensors and read out threads.
     *
     */
    if (SensorReadInit() != MSG_OK)
        osalSysHalt("Sensor initialization failed.");

    /*
     *
     * Start the System Information.
     *
     */
    SystemInformationInit();

    /*
     *
     * Start the Motion Capture Support.
     *
     */
    MotionCaptureInit();

    /*
     *
     * Start the Serial Manager.
     *
     */
    vSerialManagerInit();

    /*
     *
     * Initialize the estimation.
     *
     */
    EstimationInit();

    /*
     *
     * Initialize the controllers.
     *
     */
    ControlInit();
}

/*
 * @brief   The function containing all drivers and modules to be deinitialized.
 *          All deinitialization should be here (except internal deinits).
 *
 * @note    This will run after the request for critical tasks to terminate,
 *          but before SysTick is disabled.
 *
 * @note    Remember to do the deinitialization in reverse order from the
 *          initialization.
 */
static void vSystemDeinitList(void)
{
    /*
     *
     * Add all driver and module deinitializations here!
     *
     */


    /*
     *
     * Stop the external interrupts.
     *
     */
    extStop(&EXTD1);

    /*
     *
     * Stop the SPI for the external flash and the RF module.
     *
     */
    spiStop(&SPID1);

    /*
     *
     * Stop the I2C for the sensors.
     *
     */
    i2cStop(&I2CD2);

    /*
     *
     * Disable the serial-over-USB CDC driver.
     *
     */
    USBDisconnect();
    USBDeinit();
}

/*
 * @brief   Private function for signaling critical tasks for shutdown and
 *          waiting for them to finish execution.
 */
static void vSystemTerminateCriticalTasks(void)
{
    /* Check if the base pointer in null on not */
    if (base_subscription != NULL)
    {
        /* Base pointer is not null, search the list to find the end */
        system_critical_subscription_t *tmp_sub = base_subscription;

        /* Set all critical thread to terminate */
        while (tmp_sub != NULL)
        {
            chThdTerminate(tmp_sub->thread);
            tmp_sub = tmp_sub->next;
        }

        /* All threads signaled, wait for termination */
        tmp_sub = base_subscription;
        while (tmp_sub != NULL)
        {
            chThdWait(tmp_sub->thread);
            tmp_sub = tmp_sub->next;
        }

        /* All threads terminated, clear the list */
        base_subscription = NULL;
    }
}

/*
 * @brief   Private function for disabling the SysTick or free running counter
 *          depending on which mode is selected.
 */
static void vSystemDisableSystick(void)
{
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

    /* Free running counter mode, disable timer. */
    stStopAlarm();

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

    /* Ticked mode, disable systick. */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*
 * @brief   Used to initialize everything in the system except the kernel
 *          itself.
 *          - Starts all HW drivers, they should all be placed here.
 *          - Starts all Modules,  they should all be placed here.
 */
void vSystemInit(void)
{
    if (system_state == SYSTEM_UNINITIALIZED || system_state == SYSTEM_STOPPED)
        system_state = SYSTEM_INITIALIZING;
    else
        osalSysHalt("System is already initialized, invalid operation");

    /* Start drivers and modules */
    vSystemInitList();

    system_state = SYSTEM_RUNNING;
}

/*
 * @brief   Used to terminate all running tasks and disable the OS. All
 *          deinit of drivers and modules should be placed here.
 */
void vSystemDeinit(void)
{
    /* Starting the shutdown sequence. */
    if (system_state == SYSTEM_RUNNING)
        system_state = SYSTEM_TERMINATING;
    else
        osalSysHalt("System is not running, invalid operation");

    /* Terminate critical tasks */
    vSystemTerminateCriticalTasks();

    /* Stop drivers */
    vSystemDeinitList();

     /* Stop SysTick */
    chSysDisable();
    vSystemDisableSystick();
    chSysEnable();

    system_state = SYSTEM_STOPPED;
}

/*
 * @brief   Returns the status if a shutdown has been requested.
 *
 * @return  The current status.
 * @retval  true if a shutdown has been requested.
 * @retval  false if there is no pending shutdown.
 */
bool bSystemShutdownRequested(void)
{
    return shutdown_requested;
}

/*
 * @brief           Requests a shutdown of the OS.
 *
 * @param[in] key   Key to safe-guard the function.
 */
void vSystemRequestShutdown(uint32_t key)
{
    /* Check shutdown key */
    if (key == SYSTEM_SHUTDOWN_KEY)
    {
        /* Signal for shutdown */
        shutdown_requested = true;
    }
}

/*
 * @brief           Subscribes a critical task to the list of functions that
 *                  need safe shutdown.
 *
 * @param[in] sub   The linked list structure for the task.
 */
void vSystemCriticalTaskSubscribe(system_critical_subscription_t *sub)
{
    /* Check if the base pointer in null on not */
    if (base_subscription == NULL)
    {
        /* First subscription */
        base_subscription = sub;
    }
    else
    {
        /* Base pointer is not null, search the list to find the end */
        system_critical_subscription_t *tmp_sub = base_subscription;

        while (tmp_sub->next != NULL)
            tmp_sub = tmp_sub->next;

        /* Save the subscription at the end of the list */
        tmp_sub->next = sub;
        sub->next = NULL;
    }
}
