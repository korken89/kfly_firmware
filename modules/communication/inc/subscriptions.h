#ifndef __SUBSCRIPTIONS_H
#define __SUBSCRIPTIONS_H

#include "slip2kflypacket.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Parsing structure for the management of subscriptions.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   The port on which the command shall be (un)subscribed.
     */
    external_port_t port;
    /**
     * @brief   Which command to (un)subscribe (from)to.
     */
    kfly_command_t command;
    /**
     * @brief   If the command shall be subscribed or unsubscribed.
     *          0 denotes unsubscribe, anything else is subscribe.
     */
    uint8_t on_off;
    /**
     * @brief   The time between transmissions of the subscription in ms.
     */
    uint32_t delta_time;
} subscription_parser_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vSubscriptionsInit(void);
bool bSubscribeToCommandI(kfly_command_t command,
                          external_port_t port,
                          uint32_t delay_ms);
bool bUnsubscribeFromCommandI(kfly_command_t command, external_port_t port);
void vUnsubscribeFromAllI(void);
void vParseManageSubscription(const uint8_t *data,
                              const uint8_t size,
                              external_port_t reception_port);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief               Creates new subscription.
 *
 * @param[in] command   Command to subscribe to.
 * @param[in] port      Port to transmit the subscription on.
 * @param[in] delay_ms  Time between transmits.
 * @return              Return true if there was a free slot, else false.
 */
static inline bool bSubscribeToCommand(kfly_command_t command,
                                       external_port_t port,
                                       uint32_t delay_ms)
{
    bool result;

    osalSysLock();
    result = bSubscribeToCommandI(command, port, delay_ms);
    osalSysUnlock();

    return result;
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
static inline bool bUnsubscribeFromCommand(kfly_command_t command,
                                           external_port_t port)
{
    bool result;

    osalSysLock();
    result = bUnsubscribeFromCommandI(command, port);
    osalSysUnlock();

    return result;
}

/**
 * @brief   Removes all ongoing subscriptions.
 */
static inline void bUnsubscribeFromAll(void)
{
    osalSysLock();
    vUnsubscribeFromAllI();
    osalSysUnlock();
}


#endif
