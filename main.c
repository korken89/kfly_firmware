#include "ch.h"
#include "hal.h"
#include "system_init.h"
#include "bootloader.h"
#include "sensor_read.h"
#include "kflypacket_generators.h"

/**
 * @brief Placeholder for error messages.
 */
volatile assert_errors kfly_assert_errors;

THD_WORKING_AREA(waThreadCalibrationPrint, 1024);
EVENTSOURCE_DECL(cal_events_es);

static THD_FUNCTION(ThreadCalibrationPrint, arg)
{
    (void)arg;

    chRegSetThreadName("CalPrint");

    /* Event registration for new sensor data */
    event_listener_t el;

    /* Data holder. */
    imu_raw_data_t imu_data;

    /* Register to new data from accelerometer and gyroscope */
    chEvtRegisterMaskWithFlags(ptrGetNewDataEventSource(),
                               &el,
                               EVENT_MASK(0),
                               ACCGYRO_DATA_AVAILABLE_EVENTMASK);


    while(1)
    {
        /* Wait for new measurement data */
        chEvtWaitAny(ALL_EVENTS);

        eventflags_t flags = chEvtGetAndClearFlags(&el);

        /* Get sensor data */
        GetRawIMUData(&imu_data);

        if (flags & ACCGYRO_DATA_AVAILABLE_EVENTMASK)
        {
            /* Send the acc and gyro data. */
            //GenerateMessage(Cmd_GetRawIMUData, PORT_USB);
            GenerateMessage(Cmd_GetRawIMUData, PORT_AUX1);
        }
    }
}

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
     * Initialize all drivers and modules.
     *
     */
    vSystemInit();

    chThdCreateStatic(waThreadCalibrationPrint,
                      sizeof(waThreadCalibrationPrint),
                      HIGHPRIO - 2,
                      ThreadCalibrationPrint,
                      NULL);
    /*
     *
     * Main task loop.
     *
     */
    while(bSystemShutdownRequested() == false)
    {
        /* The "Calibration Blink"... */
        palSetPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(50);
        palClearPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(50);
        palSetPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(50);
        palClearPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(50);
        palSetPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(50);
        palClearPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(500);
        //vSystemRequestShutdown(SYSTEM_SHUTDOWN_KEY);
    }

    /*
     *
     * Deinitialize all drivers and modules.
     *
     */
    vSystemDeinit();

   /*
    *
    * All threads, drivers, interrupts and SysTick are now disabled.
    * The main function is now just a "normal" function again.
    *
    */

    /*
     *
     * Start the DFU bootloader.
     * This can be replaced if a custom bootloader is available.
     *
     */
    //vBootloaderResetAndStartDFU();

    /* In case of error get stuck here */
    while (1);
}
