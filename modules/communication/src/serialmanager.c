/* *
 *
 * OS layer for Serial Communication.
 * Handles package coding/decoding.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "myusb.h"
#include "serialmanager_types.h"
#include "statemachine.h"
#include "crc.h"
#include "serialmanager.h"

static Serial_Datapump_Holder data_pumps = {
    .ptrUSBDataPump = NULL,
    .ptrAUX1DataPump = NULL,
    .ptrAUX2DataPump = NULL,
    .ptrAUX3DataPump = NULL,
    .ptrAUX4DataPump = NULL
};

static THD_WORKING_AREA(waUSBSerialManagerTask, 128);
static THD_WORKING_AREA(waUSBDataPumpTask, 128);

/**
 * @brief      The Serial Manager task will handle incoming
 *             data and direct it for decode and processing.
 */
static THD_FUNCTION(USBSerialManagerTask, arg)
{
    (void)arg;

    /* Data structure for communication */
    static Parser_Holder_Type data_holder;

    /* Buffer for parsing serial USB commands */
    CCM_MEMORY static uint8_t buffer[SERIAL_RECIEVE_BUFFER_SIZE]; 

    /* Initialize data structure */
    vInitStatemachineDataHolder(&data_holder, PORT_USB, buffer);

    while(1)
        vStatemachineDataEntry((uint8_t)USBReadByte(TIME_INFINITE),
                               &data_holder);

    return MSG_OK;
}

/**
 * @brief               Reads a chunk of data from a circular buffer.
 *  
 * @param[in] arg       Number of bytes to read.
 */
static THD_FUNCTION(USBDataPumpTask, arg)
{
    (void)arg;

    uint8_t *read_pointer = NULL;
    uint32_t read_size;

    /* Buffer for transmitting serial USB commands */
    CCM_MEMORY static uint8_t buffer[SERIAL_TRANSMIT_BUFFER_SIZE]; 

    /* Initialize the USB transmit circular buffer */
    CircularBuffer_Init(&data_pumps.USBTransmitBuffer,
                        buffer,
                        SERIAL_TRANSMIT_BUFFER_SIZE);
    CircularBuffer_InitMutex(&data_pumps.USBTransmitBuffer);

    /* Put the USB dapa pump thread into the list of avilable data pumps */
    data_pumps.ptrUSBDataPump = chThdGetSelfX();

    while(1)
    {
        /* Wait for a start transmission event */
        chEvtWaitAny(ALL_EVENTS);

        /* We will only get here is a request to send data has been recieved */

        if (isUSBActive() == true)
        {
            /* Read out the number of bytes to send and the pointer to the
               first byte */
            read_size = CircularBuffer_GetReadPointer(
                                                &data_pumps.USBTransmitBuffer,
                                                read_pointer);

            /* Claim the USB bus during the entire transfer */
            USBClaim();
            while (read_size > 0)
            {
                /* Send the data from the circular buffer */
                USBSendData(read_pointer, read_size, TIME_INFINITE);

                /* Increment the circular buffer tail */
                CircularBuffer_IncrementTail(&data_pumps.USBTransmitBuffer,
                                             read_size);

                /* Get the read size again in case new data is avaiable or if
                   we reached the end of the buffer (to make sure the entire
                   buffer is sent) */
                read_size = CircularBuffer_GetReadPointer(
                                                &data_pumps.USBTransmitBuffer,
                                                read_pointer);

                /* If the USB has been removed during the transfer: abort */
                if (isUSBActive() == false)
                    break;
            }
            /* Release the USB bus */
            USBRelease();
        }
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

    chThdCreateStatic(waUSBDataPumpTask,
                      sizeof(waUSBDataPumpTask),
                      NORMALPRIO,
                      USBDataPumpTask,
                      NULL);
}

Circular_Buffer_Type *SerialManager_GetCircularBufferFromPort(Port_Type port)
{
    if (port == PORT_USB)
        return &data_pumps.USBTransmitBuffer;

    else if (port == PORT_AUX1)
        return &data_pumps.AUX1TransmitBuffer;

    else if (port == PORT_AUX2)
        return &data_pumps.AUX2TransmitBuffer;

    else if (port == PORT_AUX3)
        return &data_pumps.AUX3TransmitBuffer;

    else if (port == PORT_AUX4)
        return &data_pumps.AUX4TransmitBuffer;
    else
        return NULL;
}

void SerialManager_StartTransmission(Port_Type port)
{
    if ((port == PORT_USB) && (data_pumps.ptrUSBDataPump != NULL))
        chEvtSignal(data_pumps.ptrUSBDataPump, 1);

    else if ((port == PORT_AUX1) && (data_pumps.ptrAUX1DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX1DataPump, 1);

    else if ((port == PORT_AUX2) && (data_pumps.ptrAUX2DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX2DataPump, 1);

    else if ((port == PORT_AUX3) && (data_pumps.ptrAUX3DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX3DataPump, 1);

    else if ((port == PORT_AUX4) && (data_pumps.ptrAUX4DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX4DataPump, 1);
}