#include "ch.h"
#include "hal.h"
#include "system_init.h"
#include "bootloader.h"
#include "cobs.h"

/**
 * @brief Placeholder for error messages.
 */
volatile assert_errors kfly_assert_errors;

#define TEST_SIZE       64
uint8_t cobs_buffer[TEST_SIZE];
uint8_t cobs_buffer2[TEST_SIZE];
circular_buffer_t cobs_cb;
cobs_encoder_t cobs_encoder;
cobs_decoder_t cobs_decoder;

const uint8_t msg1[] = {1,2,3,4,5,6};
const uint8_t msg2[] = {1,2,3,0,5,6,7,8,9};
const uint8_t msg3[] = {1,2,3,0,0,6,7,8,9};
const uint8_t msg4[] = {1,2,3,0,0,0,7,8,9};
const uint8_t msg5[] = {1,2,3,0,0,0,0,0,0,7,8,9,0};
const uint8_t msg6[] = {1,2,3,0,0,0,0,0,0,7,8,9,0,0};
const uint8_t msg7[] = {1,2,3,0,0,0,0,0,0,7,8,9,0,0,0};
const uint8_t msg8[] = {1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,7,8,9,0,0,0,0,0,0,0,0,0};
const uint8_t msg9[] = {1,2,3,0,0,0,0,7,8,9,0,0,0};

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


    CircularBuffer_Init(&cobs_cb, cobs_buffer, TEST_SIZE);
    COBSInitDecoder(cobs_buffer2, TEST_SIZE, NULL, &cobs_decoder);

    COBSEncode(msg1, sizeof(msg1), &cobs_cb, &cobs_encoder);
    //COBSEncode(msg2, sizeof(msg2), &cobs_cb, &cobs_encoder);
    //COBSEncode(msg3, sizeof(msg3), &cobs_cb, &cobs_encoder);

    while(cobs_cb.tail < cobs_cb.head)
    {
        COBSDecode(CircularBuffer_ReadSingle(&cobs_cb), &cobs_decoder);
    }

    /*
     *
     * Initialize all drivers and modules.
     *
     */
    //vSystemInit();

    /*
     *
     * Main task loop.
     *
     */
    while(bSystemShutdownRequested() == false)
    {
        palSetPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(150);
        palClearPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(150);
        palSetPad(GPIOC, GPIOC_LED_USR);
        chThdSleepMilliseconds(150);
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
