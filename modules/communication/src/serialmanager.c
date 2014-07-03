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
#include "statemachine_generators.h"
#include "crc.h"
#include "serialmanager.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define START_TRANSMISSION_EVENT                                EVENT_MASK(0)

#define AUX1_SERIAL_DRIVER                                      SD3
#define AUX2_SERIAL_DRIVER                                      SD5
#define AUX3_SERIAL_DRIVER                                      SD4

#define MAX_NUMBER_OF_SUBSCRIPTIONS                             10

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
} Serial_Datapump_Holder;

/**
 * @brief   Holder of the necessary information for each
 *          message subscription slot.
 */
typedef struct
{
    /**
     * @brief   Virtual timer taking care of the periodicity of the message.
     */
    virtual_timer_t vt;
    /**
     * @brief   Time between messages in milliseconds.
     */
    uint32_t delay_ms;
    /**
     * @brief   The command to be sent at each timeout.
     */
    KFly_Command command;
    /**
     * @brief   The port for the message to be sent on.
     */
    External_Port port;
} subscription_slot_t;

/**
 * @brief   Subscriptions structure. Contains the subscription slots and 
 *          the mailboxes.
 */
typedef struct
{
    /**
     * @brief   Message transmission mailbox.
     */
    mailbox_t mb;
    /**
     * @brief   Mailbox message holder.
     */
    msg_t box[2 * MAX_NUMBER_OF_SUBSCRIPTIONS];
    /**
     * @brief   Subscription slots.
     */
    subscription_slot_t slot[MAX_NUMBER_OF_SUBSCRIPTIONS];
} subscription_t;

/* Instance of the data pump holder structure */
Serial_Datapump_Holder data_pumps = {
    .ptrUSBDataPump = NULL,
    .ptrAUX1DataPump = NULL,
    .ptrAUX2DataPump = NULL,
    .ptrAUX3DataPump = NULL,
    .ptrAUX4DataPump = NULL
};

/* Temporary settings until serial is settable through the USB */
static const SerialConfig aux1_config =
{
  115200,
  0,
  USART_CR2_STOP2_BITS,
  0
};

/* Subscription holder */
subscription_t subscriptions;

/*===================================================*/
/* Working area for the data pump                    */
/* and data decode threads.                          */
/*===================================================*/

THD_WORKING_AREA(waUSBSerialManagerTask, 128);
THD_WORKING_AREA(waUSBDataPumpTask, 128);

THD_WORKING_AREA(waAux1SerialManagerTask, 128);
THD_WORKING_AREA(waAux1DataPumpTask, 128);
/* TODO: Add for the rest of the communication interfaces */

THD_WORKING_AREA(waSubscriptionsTask, 128);

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
__attribute__((noreturn))
static THD_FUNCTION(USBSerialManagerTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("USB Serial Manager");

    /* Data structure for communication */
    static parser_holder_t data_holder;

    /* Buffer for parsing serial USB commands */
    CCM_MEMORY static uint8_t USB_in_buffer[SERIAL_RECIEVE_BUFFER_SIZE]; 

    /* Initialize data structure */
    vInitStatemachineDataHolder(&data_holder, PORT_USB, USB_in_buffer);

    while(1)
        vStatemachineDataEntry(USBReadByte(TIME_INFINITE),
                               &data_holder);
}

/**
 * @brief           Transmits the content of the USB circular buffer over USB.
 *  
 * @param[in] arg   Input argument (unused).
 */
__attribute__((noreturn))
static THD_FUNCTION(USBDataPumpTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("USB Data Pump");

    /* Buffer for transmitting serial USB commands */
    CCM_MEMORY static uint8_t USB_out_buffer[SERIAL_TRANSMIT_BUFFER_SIZE]; 

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
__attribute__((noreturn))
static THD_FUNCTION(Aux1SerialManagerTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("Aux1 Serial Manager");

    /* Data structure for communication */
    static parser_holder_t data_holder;

    /* Buffer for parsing serial USB commands */
    CCM_MEMORY static uint8_t AUX1_in_buffer[SERIAL_RECIEVE_BUFFER_SIZE]; 

    /* Initialize data structure */
    vInitStatemachineDataHolder(&data_holder, PORT_AUX1, AUX1_in_buffer);

    while(1)
        vStatemachineDataEntry(sdGet(&AUX1_SERIAL_DRIVER),
                               &data_holder);
}

/**
 * @brief           Transmits the content of the Aux1 circular buffer over Aux1.
 *  
 * @param[in] arg   Input argument (unused).
 */
__attribute__((noreturn))
static THD_FUNCTION(Aux1DataPumpTask, arg)
{
    (void)arg;

    /* Name for debug */
    chRegSetThreadName("Aux1 Data Pump");

    /* Buffer for transmitting serial Aux1 commands */
    CCM_MEMORY static uint8_t AUX1_out_buffer[SERIAL_TRANSMIT_BUFFER_SIZE]; 

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

/*===================================================*/
/* Message Subscription thread.                      */
/*===================================================*/

/**
 * @brief           Keeps track of the current subscriptions.
 *  
 * @param[in] arg   Input argument (unused).
 */
__attribute__((noreturn))
static THD_FUNCTION(SubscriptionsTask, arg)
{
    (void)arg;

    msg_t message;
    subscription_slot_t *slot;

    /* Name for debug */
    chRegSetThreadName("Subscriptions");

    while(1)
    {
        /* Wait for a new message */
        chMBFetch(&subscriptions.mb, &message, TIME_INFINITE);
        slot = (subscription_slot_t *)message;

        /* Transmit the message */
        if (GenerateMessage(slot->command, slot->port) != HAL_SUCCESS)
        {
            /* Transmission buffer full */
        }
    }
}

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
    uint32_t read_size;

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

/**
 * @brief   Initializes the subscription structures.
 */
static void SubscriptionsInit(void)
{
    int i;

    /* Initialize the mailbox that passes the requested messages */
    chMBObjectInit(&subscriptions.mb,
                   subscriptions.box,
                   2 * MAX_NUMBER_OF_SUBSCRIPTIONS);

    /* Initialize the subscription array */
    for (i = 0; i < MAX_NUMBER_OF_SUBSCRIPTIONS; i++)
    {
        chVTObjectInit(&subscriptions.slot[i].vt);
        subscriptions.slot[i].command = Cmd_None;
    }
}

/**
 * @brief           Subscription virtual timer callback.
 *             
 * @param[in] p     Pointer to the subscription structure that requested
 *                  the callback.
 */
static void SubscriptionCallback(void *p)
{
    osalSysLockFromISR();

    /* Restart the virtual timer for the next message */
    chVTSetI(&((subscription_slot_t *)p)->vt,
             MS2ST(((subscription_slot_t *)p)->delay_ms),
             SubscriptionCallback,
             p);

    /* Send the message to the transmission thread */
    if (chMBPostI(&subscriptions.mb, (msg_t)p) == MSG_TIMEOUT)
    {
        /* Error! Mailbox is full. Disable all subscriptions. */
        UnsubscribeFromAllI();
    }

    osalSysUnlockFromISR();
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

    /* Initialize the subscription structure */
    SubscriptionsInit();

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

    /* Start the subscriptions task */
    chThdCreateStatic(waSubscriptionsTask,
                      sizeof(waSubscriptionsTask),
                      NORMALPRIO,
                      SubscriptionsTask,
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
circular_buffer_t *SerialManager_GetCircularBufferFromPort(External_Port port)
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
void SerialManager_StartTransmission(External_Port port)
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

/*=====================================*/
/* Message subscription functionality. */
/*=====================================*/

/**
 * @brief               Creates new subscription. I-class function.
 *             
 * @param[in] command   Command to subscribe to.
 * @param[in] port      Port to transmit the subscription on.
 * @param[in] delay_ms  Time between transmits.
 * @return              Return true if there was a free slot, else false.
 */
bool SubscribeToCommandI(KFly_Command command,
                        External_Port port,
                        uint32_t delay_ms)
{
    int i;

    /* Look for a free subscription slot */
    for (i = 0; i < MAX_NUMBER_OF_SUBSCRIPTIONS; i++)
    {
        if ((subscriptions.slot[i].command == command) &&
            (subscriptions.slot[i].port == port))
        {
            /* The subscription already exists. Change the current timebase
               to the new subscription. */
            subscriptions.slot[i].delay_ms = delay_ms;

            chVTSetI(&subscriptions.slot[i].vt,
                     MS2ST(delay_ms),
                     SubscriptionCallback,
                     &subscriptions.slot[i]);

            return true;
        }
        else if (subscriptions.slot[i].command == Cmd_None)
        {
            /* Free subscription slot! Set the structure and initialize
               the virtual timer. */
            subscriptions.slot[i].command = command;
            subscriptions.slot[i].port = port;
            subscriptions.slot[i].delay_ms = delay_ms;

            chVTSetI(&subscriptions.slot[i].vt,
                     MS2ST(delay_ms),
                     SubscriptionCallback,
                     &subscriptions.slot[i]);

            return true;
        }
    }

    /* No free subscription slots */
    return false;
}

/**
 * @brief               Creates new subscription.
 *             
 * @param[in] command   Command to subscribe to.
 * @param[in] port      Port to transmit the subscription on.
 * @param[in] delay_ms  Time between transmits.
 * @return              Return true if there was a free slot, else false.
 */
bool SubscribeToCommand(KFly_Command command,
                        External_Port port,
                        uint32_t delay_ms)
{
    bool result;

    osalSysLock();
    result = SubscribeToCommandI(command, port, delay_ms);
    osalSysUnlock();

    return result;
}

/**
 * @brief               Removes a subscription from a port. I-class function.
 *             
 * @param[in] command   Command to unsubscribe from.
 * @param[in] port      Port to unsubscribe from.
 * @return              Returns true if the subscription was successfully
 *                      deleted. False indicates it did not find any
 *                      subscription by the specified command.
 */
bool UnsubscribeFromCommandI(KFly_Command command, External_Port port)
{
    int i;

    /* Search for the subscription to delete */
    for (i = 0; i < MAX_NUMBER_OF_SUBSCRIPTIONS; i++)
    {
        if ((subscriptions.slot[i].command == command) &&
            (subscriptions.slot[i].port == port))
        {
            /* Disable the timer and reset the command */
            chVTResetI(&subscriptions.slot[i].vt);
            subscriptions.slot[i].command = Cmd_None;

            return true;
        }
    }

    /* No subscription was found with the requested command */
    return false;
}

/**
 * @brief               Removes a subscription from a port.
 *             
 * @param[in] command   Command to unsubscribe from.
 * @param[in] port      Port to unsubscribe from.
 * @return              Returns true if the subscription was successfully
 *                      deleted. False indicates it did not find any
 *                      subscription by the specified command.
 */
bool UnsubscribeFromCommand(KFly_Command command, External_Port port)
{
    bool result;

    osalSysLock();
    result = UnsubscribeFromCommandI(command, port);
    osalSysUnlock();

    return result;
}

/**
 * @brief   Removes all ongoing subscriptions. I-class function.
 */
void UnsubscribeFromAllI(void)
{
    int i;

    /* Delete all subscriptions */
    for (i = 0; i < MAX_NUMBER_OF_SUBSCRIPTIONS; i++)
    {
        if (subscriptions.slot[i].command != Cmd_None)
        {
            /* Disable the trimer and reset the command */
            chVTResetI(&subscriptions.slot[i].vt);
            subscriptions.slot[i].command = Cmd_None;
        }
    }
}

/**
 * @brief   Removes all ongoing subscriptions.
 */
void UnsubscribeFromAll(void)
{
    osalSysLock();
    UnsubscribeFromAllI();
    osalSysUnlock();
}
