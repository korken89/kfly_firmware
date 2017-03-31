/* *
 *
 * OS layer for Serial Communication.
 * Handles package coding/decoding.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "usb_access.h"
#include "slip.h"
#include "slip2kflypacket.h"
#include "kflypacket_generators.h"
#include "crc.h"
#include "serialmanager.h"
#include "subscriptions.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define START_TRANSMISSION_EVENT            EVENT_MASK(0)

#define AUX1_SERIAL_DRIVER                  SD3
#define AUX2_SERIAL_DRIVER                  SD5
#define AUX3_SERIAL_DRIVER                  SD4

static bool USBTransmitCircularBuffer(circular_buffer_t *Cbuff);
static bool AuxTransmitCircularBuffer(SerialDriver *sdp,
                                      circular_buffer_t *Cbuff);

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Holder of the necessary information for the data pump threads.
 */
typedef struct
{
    /**
     * @brief   Pointer to the USB data pump thread.
     */
    thread_t *ptrUSBDataPump;
    /**
     * @brief   USB data pump circular transmit buffer.
     */
    circular_buffer_t USBTransmitBuffer;
    /**
     * @brief   Pointer to the AUX1 data pump thread.
     */
    thread_t *ptrAUX1DataPump;
    /**
     * @brief   AUX1 data pump circular transmit buffer.
     */
    circular_buffer_t AUX1TransmitBuffer;
    /**
     * @brief   Pointer to the AUX2 data pump thread.
     */
    thread_t *ptrAUX2DataPump;
    /**
     * @brief   AUX2 data pump circular transmit buffer.
     */
    circular_buffer_t AUX2TransmitBuffer;
    /**
     * @brief   Pointer to the AUX3 data pump thread.
     */
    thread_t *ptrAUX3DataPump;
    /**
     * @brief   AUX3 data pump circular transmit buffer.
     */
    circular_buffer_t AUX3TransmitBuffer;
    /**
     * @brief   Pointer to the AUX4 data pump thread.
     */
    thread_t *ptrAUX4DataPump;
    /**
     * @brief   AUX4 data pump circular transmit buffer.
     */
    circular_buffer_t AUX4TransmitBuffer;
} serial_datapump_holder_t;

/* Instance of the data pump holder structure */
serial_datapump_holder_t data_pumps = {
    .ptrUSBDataPump = NULL,
    .ptrAUX1DataPump = NULL,
    .ptrAUX2DataPump = NULL,
    .ptrAUX3DataPump = NULL,
    .ptrAUX4DataPump = NULL
};

/* Temporary settings until serial is settable through the USB */
static const SerialConfig aux1_config =
{
  1000000,
  0,
  USART_CR2_STOP1_BITS,
  0
};


/*===================================================*/
/* Working area for the data pump                    */
/* and data decode threads.                          */
/*===================================================*/

THD_WORKING_AREA(waUSBSerialManagerTask, 256);
THD_WORKING_AREA(waUSBDataPumpTask, 256);

THD_WORKING_AREA(waAux1SerialManagerTask, 256);
THD_WORKING_AREA(waAux1DataPumpTask, 256);
/* TODO: Add for the rest of the communication interfaces */

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===================================================*/
/* USB Communication threads.                        */
/*===================================================*/

/**
 * @brief           The USB Serial Manager task will handle incoming
 *                  data and direct it for decode and processing.
 *
 * @param[in] arg   Input argument (unused).
 */
static THD_FUNCTION(USBSerialManagerTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("USB Serial Manager");

    /* Data structure for communication */
    static slip_parser_t slip_data_holder;
    static kfly_parser_t kfly_data_holder;

    /* Anonymous function for connecting the SLIP parser to the KFly parser. */
    void bind(slip_parser_t *p)
    {
        ParseKFlyPacketFromSLIP(p, &kfly_data_holder);
    }

    /* Buffer for parsing serial USB commands */
    static uint8_t USB_in_buffer[SERIAL_RECIEVE_BUFFER_SIZE];

    /* Initialize data structures */
    InitSLIPParser(&slip_data_holder,
                   USB_in_buffer,
                   SERIAL_RECIEVE_BUFFER_SIZE,
                   bind);

    /* Cut away the header. */
    InitKFlyPacketParser(&kfly_data_holder, PORT_USB, &USB_in_buffer[2]);

    while(1)
    {
        /* Check so the USB is available, else wait a little */
        while (isUSBActive() == false)
            chThdSleepMilliseconds(200);

        /* Pump data into the SLIP parser. */
        ParseSLIP(USBReadByte(TIME_INFINITE), &slip_data_holder);
    }
}

/**
 * @brief           Transmits the content of the USB circular buffer over USB.
 *
 * @param[in] arg   Input argument (unused).
 */
static THD_FUNCTION(USBDataPumpTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("USB Data Pump");

    /* Buffer for transmitting serial USB commands */
    static uint8_t USB_out_buffer[SERIAL_TRANSMIT_BUFFER_SIZE];

    /* Initialize the USB transmit circular buffer */
    CircularBuffer_Init(&data_pumps.USBTransmitBuffer,
                        USB_out_buffer,
                        SERIAL_TRANSMIT_BUFFER_SIZE);
    CircularBuffer_InitMutex(&data_pumps.USBTransmitBuffer);

    /* Put the USB data pump thread into the list of available data pumps */
    data_pumps.ptrUSBDataPump = chThdGetSelfX();

    while(1)
    {
        /* Wait for a start transmission event */
        chEvtWaitAny(START_TRANSMISSION_EVENT);

        /* We will only get here is a request to send data has been received */
        USBTransmitCircularBuffer(&data_pumps.USBTransmitBuffer);
    }
}

/*===================================================*/
/* AUX1 Communication threads.                       */
/*===================================================*/

/**
 * @brief           The Aux1 Serial Manager task will handle incoming
 *                  data and direct it for decode and processing.
 *
 * @param[in] arg   Input argument (unused).
 */
static THD_FUNCTION(Aux1SerialManagerTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("Aux1 Serial Manager");

    /* Data structure for communication */
    static slip_parser_t slip_data_holder;
    static kfly_parser_t kfly_data_holder;

    /* Anonymous function for connecting the SLIP parser to the KFly parser. */
    void bind(slip_parser_t *p)
    {
        ParseKFlyPacketFromSLIP(p, &kfly_data_holder);
    }

    /* Buffer for parsing serial commands */
    static uint8_t AUX1_in_buffer[SERIAL_RECIEVE_BUFFER_SIZE];

    /* Initialize data structures */
    InitSLIPParser(&slip_data_holder,
                   AUX1_in_buffer,
                   SERIAL_RECIEVE_BUFFER_SIZE,
                   bind);

    /* Cut away the header. */
    InitKFlyPacketParser(&kfly_data_holder, PORT_AUX1, &AUX1_in_buffer[2]);

    while(1)
    {
        /* Pump data into the SLIP parser. */
        ParseSLIP(sdGet(&AUX1_SERIAL_DRIVER), &slip_data_holder);
    }
}

/**
 * @brief           Transmits the content of the Aux1 circular buffer over Aux1.
 *
 * @param[in] arg   Input argument (unused).
 */
static THD_FUNCTION(Aux1DataPumpTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("Aux1 Data Pump");

    /* Buffer for transmitting serial Aux1 commands */
    static uint8_t AUX1_out_buffer[SERIAL_TRANSMIT_BUFFER_SIZE];

    /* Initialize the Aux1 transmit circular buffer */
    CircularBuffer_Init(&data_pumps.AUX1TransmitBuffer,
                        AUX1_out_buffer,
                        SERIAL_TRANSMIT_BUFFER_SIZE);
    CircularBuffer_InitMutex(&data_pumps.AUX1TransmitBuffer);

    /* Put the Aux1 data pump thread into the list of available data pumps */
    data_pumps.ptrAUX1DataPump = chThdGetSelfX();

    while(1)
    {
        /* Wait for a start transmission event */
        chEvtWaitAny(START_TRANSMISSION_EVENT);

        /* We will only get here is a request to send data has been received */
        AuxTransmitCircularBuffer(&AUX1_SERIAL_DRIVER,
                                  &data_pumps.AUX1TransmitBuffer);
    }
}

/*===================================================*/
/* AUX2 Communication threads.                       */
/*===================================================*/

/* To be added */


/*===================================================*/
/* AUX3 Communication threads.                       */
/*===================================================*/

/* To be added */


/*===================================================*/
/* AUX4 Communication threads.                       */
/*===================================================*/

/* To be added */


/*===================================================*/
/* RF Communication threads.                         */
/*===================================================*/

/* To be added */


/**
 * @brief               Transmits a circular buffer over the USB interface.
 *
 * @param[in] Cbuff     Circular buffer to transmit.
 * @return              Returns HAL_FAILED if it did not succeed to transmit
 *                      the buffer, else HAL_SUCCESS is returned.
 */
static bool USBTransmitCircularBuffer(circular_buffer_t *Cbuff)
{
    uint8_t *read_pointer;
    size_t read_size;

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
    else /* Some error occurred */
        return HAL_FAILED;
}

/**
 * @brief               Transmits a circular buffer over the UART (Aux)
 *                      interface.
 *
 * @param[in] sdp       Pointer to the Serial Driver to transmit the data over.
 * @param[in] Cbuff     Circular buffer to transmit.
 * @return              Returns HAL_FAILED if it did not succeed to transmit
 *                      the buffer, else HAL_SUCCESS is returned.
 */
static bool AuxTransmitCircularBuffer(SerialDriver *sdp,
                                      circular_buffer_t *Cbuff)
{
    uint8_t *read_pointer;
    size_t read_size;

    if ((sdp != NULL) && (Cbuff != NULL))
    {
        /* Read out the number of bytes to send and the pointer to the
           first byte */
        read_pointer = CircularBuffer_GetReadPointer(Cbuff, &read_size);

        while (read_size > 0)
        {
            /* Send the data from the circular buffer */
            sdWrite(sdp, read_pointer, read_size);

            /* Increment the circular buffer tail */
            CircularBuffer_IncrementTail(Cbuff, read_size);

            /* Get the read size again in case new data is available or if
               we reached the end of the buffer (to make sure the entire
               buffer is sent) */
            read_pointer = CircularBuffer_GetReadPointer(Cbuff, &read_size);
        }

        /* Transfer finished successfully */
        return HAL_SUCCESS;
    }
    else /* Some error occurred */
        return HAL_FAILED;
}


/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes communication.
 */
void vSerialManagerInit(void)
{
    /* Initialize the USB mutex */
    USBMutexInit();

    /* Initialize the subscription subsystem */
    vSubscriptionsInit();

    /* Start the USB communication tasks */

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

    /* Start the Aux1 communication tasks */

    sdStart(&AUX1_SERIAL_DRIVER, &aux1_config);

    chThdCreateStatic(waAux1SerialManagerTask,
                      sizeof(waAux1SerialManagerTask),
                      NORMALPRIO,
                      Aux1SerialManagerTask,
                      NULL);

    chThdCreateStatic(waAux1DataPumpTask,
                      sizeof(waAux1DataPumpTask),
                      NORMALPRIO,
                      Aux1DataPumpTask,
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
circular_buffer_t *SerialManager_GetCircularBufferFromPort(external_port_t port)
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
void SerialManager_StartTransmission(external_port_t port)
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
