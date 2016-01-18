#ifndef __KFLYPACKET_PARSERS_H
#define __KFLYPACKET_PARSERS_H

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
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
kfly_data_parser_t GetParser(kfly_command_t command);

#endif
