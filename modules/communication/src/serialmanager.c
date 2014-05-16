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

/*===========================================================================*/
/* Working area for the data pump and data decode threads.                   */
/*===========================================================================*/

static THD_WORKING_AREA(waUSBSerialManagerTask, 128);
static THD_WORKING_AREA(waUSBDataPumpTask, 128);
/* TODO: Add for the rest of the communication interfaces */


/*===========================================================================*/
/* USB Communication threads.                                                */
/*===========================================================================*/

/**
 * @brief               The Serial Manager task will handle incoming
 *                      data and direct it for decode and processing.
 *             
 * @param[in] arg       Input argument (unused).
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
 * @brief               Transmits the content of the USB circular buffer
 *                      over USB.
 *  
 * @param[in] arg       Input argument (unused).
 */
static THD_FUNCTION(USBDataPumpTask, arg)
{
    (void)arg;

    /* Buffer for transmitting serial USB commands */
    CCM_MEMORY static uint8_t buffer[SERIAL_TRANSMIT_BUFFER_SIZE]; 

    /* Initialize the USB transmit circular buffer */
    CircularBuffer_Init(&data_pumps.USBTransmitBuffer,
                        buffer,
                        SERIAL_TRANSMIT_BUFFER_SIZE);
    CircularBuffer_InitMutex(&data_pumps.USBTransmitBuffer);

    /* Put the USB data pump thread into the list of available data pumps */
    data_pumps.ptrUSBDataPump = chThdGetSelfX();

    while(1)
    {
        /* Wait for a start transmission event */
        chEvtWaitAny(START_TRANSMISSION_EVENT);

        /* We will only get here is a request to send data has been received */
        SerialManager_USBTransmitCircularBuffer(&data_pumps.USBTransmitBuffer);
    }

    return MSG_OK;
}

/*===========================================================================*/
/* AUX1 Communication threads.                                               */
/*===========================================================================*/

/* To be added */


/*===========================================================================*/
/* AUX2 Communication threads.                                               */
/*===========================================================================*/

/* To be added */


/*===========================================================================*/
/* AUX3 Communication threads.                                               */
/*===========================================================================*/

/* To be added */


/*===========================================================================*/
/* AUX4 Communication threads.                                               */
/*===========================================================================*/

/* To be added */


/*===========================================================================*/
/* RF Communication threads.                                                 */
/*===========================================================================*/

/* To be added */


/*===========================================================================*/
/* Exported functions.                                                       */
/*===========================================================================*/

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

/**
 * @brief               Return the circular buffer of corresponding 
 *                      communication port.
 *             
 * @param[in] port      Port parameter.
 * @return              Returns the pointer to the corresponding port's 
 *                      circular buffer.
 */
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

/**
 * @brief               Signal the data pump thread to start transmission.
 *             
 * @param[in] port      Port parameter.
 */
void SerialManager_StartTransmission(Port_Type port)
{
    if ((port == PORT_USB) && (data_pumps.ptrUSBDataPump != NULL))
        chEvtSignal(data_pumps.ptrUSBDataPump, START_TRANSMISSION_EVENT);

    else if ((port == PORT_AUX1) && (data_pumps.ptrAUX1DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX1DataPump, START_TRANSMISSION_EVENT);

    else if ((port == PORT_AUX2) && (data_pumps.ptrAUX2DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX2DataPump, START_TRANSMISSION_EVENT);

    else if ((port == PORT_AUX3) && (data_pumps.ptrAUX3DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX3DataPump, START_TRANSMISSION_EVENT);

    else if ((port == PORT_AUX4) && (data_pumps.ptrAUX4DataPump != NULL))
        chEvtSignal(data_pumps.ptrAUX4DataPump, START_TRANSMISSION_EVENT);
}

/**
 * @brief               Transmits a circular buffer over the USB interface.
 *             
 * @param[in] Cbuff     Circular buffer to transmit.
 * @return              Returns HAL_FAILED if it did not succeed to transmit
 *                      the buffer, else HAL_SUCCESS is returned.
 */
bool SerialManager_USBTransmitCircularBuffer(Circular_Buffer_Type *Cbuff)
{
    uint8_t *read_pointer;
    uint32_t read_size;

    if ((isUSBActive() == true) && (Cbuff != NULL))
    {
        /* Read out the number of bytes to send and the pointer to the
           first byte */
        read_pointer = CircularBuffer_GetReadPointer(Cbuff, &read_size);

        /* Claim the USB bus during the entire transfer */
        USBClaim();
        while (read_size > 0)
        {
            /* Send the data from the circular buffer */
            USBSendData(read_pointer, read_size, TIME_INFINITE);

            /* Increment the circular buffer tail */
            CircularBuffer_IncrementTail(Cbuff, read_size);

            /* Get the read size again in case new data is available or if
               we reached the end of the buffer (to make sure the entire
               buffer is sent) */
            read_pointer = CircularBuffer_GetReadPointer(Cbuff, &read_size);

            /* If the USB has been removed during the transfer: abort */
            if (isUSBActive() == false)
                return HAL_FAILED;
        }
        /* Release the USB bus */
        USBRelease();

        /* Transfer finished successfully */
        return HAL_SUCCESS;
    }
    else
        return HAL_FAILED;
}