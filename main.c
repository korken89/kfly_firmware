#include "ch.h"
#include "hal.h"
#include "system_init.h"
#include "bootloader.h"

/**
 * @brief Placeholder for error messages.
 */
volatile assert_errors kfly_assert_errors;

static system_critical_subscription_t testSub1;

THD_WORKING_AREA(waThreadTest1, 256);
static THD_FUNCTION(ThreadTest1, arg)
{
    (void)arg;

    int i;

    chRegSetThreadName("Test1");

    testSub1.thread = chThdGetSelfX();

    vSystemCriticalTaskSubscribe(&testSub1);

    while(chThdShouldTerminateX() == false)
    {
        palTogglePad(GPIOC, GPIOC_LED_ERR);
        chThdSleepMilliseconds(100);
    }

    for (i = 0; i < 6; i++)
    {
        palTogglePad(GPIOC, GPIOC_LED_ERR);
        chThdSleepMilliseconds(500);
    }

    return 0;
}



static system_critical_subscription_t testSub2;

THD_WORKING_AREA(waThreadTest2, 256);
static THD_FUNCTION(ThreadTest2, arg)
{
    (void)arg;

    int i;

    chRegSetThreadName("Test2");

    testSub2.thread = chThdGetSelfX();

    vSystemCriticalTaskSubscribe(&testSub2);

    while(chThdShouldTerminateX() == false)
    {
        palTogglePad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(100);
    }

    for (i = 0; i < 6; i++)
    {
        palTogglePad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(500);
    }

    return 0;
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

    chThdCreateStatic(waThreadTest1,
                      sizeof(waThreadTest1),
                      NORMALPRIO,
                      ThreadTest1,
                      NULL);

    chThdCreateStatic(waThreadTest2,
                      sizeof(waThreadTest2),
                      NORMALPRIO,
                      ThreadTest2,
                      NULL);

    /*
     *
     * Idle task loop.
     *
     */
    while(bSystemShutdownRequested() == false)
    {
        //palTogglePad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(5000);
        vSystemRequestShutdown(SYSTEM_SHUTDOWN_KEY);
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
    vBootloaderResetAndStartDFU();

    /* In case of error get stuck here */
    while (1);
}
