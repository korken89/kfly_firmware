#ifndef __STATEMACHINE_PARSERS_H
#define __STATEMACHINE_PARSERS_H

#include "statemachine.h"

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
    External_Port port;
    /**
     * @brief   Which command to (un)subscribe (from)to.
     */
    KFly_Command command;
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
parser_t GetParser(KFly_Command command);

#endif
