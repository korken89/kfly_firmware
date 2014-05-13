/* *
 *
 * OS layer for Serial Communication.
 * Handles package coding/decoding.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "circularbuffer.h"
#include "serialmanager_types.h"
#include "statemachine.h"
#include "crc.h"
#include "serialmanager.h"

static THD_WORKING_AREA(waUSBSerialManagerTask, 128);

/**
 * @brief              The Serial Manager task will handle incoming
 *                     data and direct it for decode and processing.
 */
static THD_FUNCTION(USBSerialManagerTask, arg)
{
    (void)arg;

    static Parser_Holder_Type data_holder;
    static uint8_t buffer[SERIAL_BUFFER_SIZE]; /* Buffer for serial USB commands */

    /* Initialize data structure */
    vInitStatemachineDataHolder(&data_holder, PORT_USB, buffer);

    while(1)
    {
        vStatemachineDataEntry(USBReadByte(), &data_holder);
    }

    return MSG_OK;
}

/**
 * @brief              Initializes communication.
 */
void vSerialManagerInit(void)
{
    USBMutexInit();

    chThdCreateStatic(waUSBSerialManagerTask,
                      sizeof(waUSBSerialManagerTask),
                      NORMALPRIO,
                      USBSerialManagerTask,
                      NULL);
}