/* *
 *
 * OS layer for Message Subscription.
 *
 * */

#include "ch.h"
#include "hal.h"
#include "statemachine_generators.h"
#include "subscriptions.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define MAX_NUMBER_OF_SUBSCRIPTIONS         10
#define SUBSCRIPTION_MAILBOX_SIZE           (MAX_NUMBER_OF_SUBSCRIPTIONS * 2)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/* Working area for the subscriptions task. */
THD_WORKING_AREA(waSubscriptionsTask, 128);

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
     * @brief   Pointer to message transmission mailbox.
     */
    mailbox_t *mb;
    /**
     * @brief   Mailbox message holder.
     */
    msg_t box[SUBSCRIPTION_MAILBOX_SIZE];
    /**
     * @brief   Subscription slots.
     */
    subscription_slot_t slot[MAX_NUMBER_OF_SUBSCRIPTIONS];
} subscription_t;

/*===================================================*/
/* Subscription mailbox and data holders             */
/*===================================================*/
subscription_t subscriptions;
MAILBOX_DECL(subscription_mailbox,
             subscriptions.box,
             SUBSCRIPTION_MAILBOX_SIZE);

/*===================================================*/
/* Working area for the data pump                    */
/* and data decode threads.                          */
/*===================================================*/

THD_WORKING_AREA(waSubscriptionsTask, 128);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===================================================*/
/* Message Subscription thread.                      */
/*===================================================*/

/**
 * @brief           Keeps track of the current subscriptions.
 *  
 * @param[in] arg   Input argument (unused).
 */
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
        chMBFetch(subscriptions.mb, &message, TIME_INFINITE);
        slot = (subscription_slot_t *)message;

        /* Transmit the message */
        if (GenerateMessage(slot->command, slot->port) != HAL_SUCCESS)
        {
            /* Transmission buffer full */
        }
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
    if (chMBPostI(subscriptions.mb, (msg_t)p) == MSG_TIMEOUT)
    {
        /* Error! Mailbox is full. Disable all subscriptions. */
        vUnsubscribeFromAllI();
    }

    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the subscription structures.
 */
void vSubscriptionsInit(void)
{
    int i;

    /* Link the mailbox to its pointer */
    subscriptions.mb = &subscription_mailbox;

    /* Initialize the mailbox that passes the requested messages */
    chMBObjectInit(subscriptions.mb,
                   subscriptions.box,
                   SUBSCRIPTION_MAILBOX_SIZE);

    /* Initialize the subscription array */
    for (i = 0; i < MAX_NUMBER_OF_SUBSCRIPTIONS; i++)
    {
        chVTObjectInit(&subscriptions.slot[i].vt);
        subscriptions.slot[i].command = Cmd_None;
    }

    /* Start the subscriptions task */
    chThdCreateStatic(waSubscriptionsTask,
                      sizeof(waSubscriptionsTask),
                      NORMALPRIO,
                      SubscriptionsTask,
                      NULL);
}

/**
 * @brief               Creates new subscription. I-class function.
 *             
 * @param[in] command   Command to subscribe to.
 * @param[in] port      Port to transmit the subscription on.
 * @param[in] delay_ms  Time between transmits.
 * @return              Return true if there was a free slot, else false.
 */
bool bSubscribeToCommandI(KFly_Command command,
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
 * @brief               Removes a subscription from a port. I-class function.
 *             
 * @param[in] command   Command to unsubscribe from.
 * @param[in] port      Port to unsubscribe from.
 * @return              Returns true if the subscription was successfully
 *                      deleted. False indicates it did not find any
 *                      subscription by the specified command.
 */
bool bUnsubscribeFromCommandI(KFly_Command command, External_Port port)
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
 * @brief   Removes all ongoing subscriptions. I-class function.
 */
void vUnsubscribeFromAllI(void)
{
    int i;

    /* Delete all subscriptions */
    for (i = 0; i < MAX_NUMBER_OF_SUBSCRIPTIONS; i++)
    {
        /* Disable the timer and reset the command */
        chVTResetI(&subscriptions.slot[i].vt);
        subscriptions.slot[i].command = Cmd_None;
    }
}
