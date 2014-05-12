/* *
 *
 * OS layer for Serial Communication.
 * Handles package coding/decoding.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "statemachine.h"
#include "crc.h"
#include "circularbuffer.h"
#include "serialmanager.h"

/**
 * @brief              Initializes communication.
 */
void vSerialManagerInit(void)
{
}

/**
 * @brief              The Serial Manager task will handle incoming
 *                     data and direct it for decode and processing.
 */
void vTaskUSBSerialManager(void *pvParameters)
{
    uint8_t in_data;
    Parser_Holder_Type data_holder;
    static uint8_t buffer[SERIAL_BUFFER_SIZE]; /* Buffer for serial USB commands */

    /* Initialize data structure */
    data_holder.Port = PORT_USB;
    data_holder.buffer = buffer;
    data_holder.current_state = NULL;
    data_holder.next_state = vWaitingForSYNC;
    data_holder.parser = NULL;
    data_holder.rx_error = 0;

    while(1)
    {
        //xQueueReceive(xUSBQueue.xUSBQueueHandle, &in_data, portMAX_DELAY);

        vStatemachineDataEntry(in_data, &data_holder);
    }
}