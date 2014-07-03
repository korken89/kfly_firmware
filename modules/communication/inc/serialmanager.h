#ifndef __SERIALMANAGER_H
#define __SERIALMANAGER_H

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
} subscriptions_parser_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vSerialManagerInit(void);
void vTaskUSBSerialManager(void *);
circular_buffer_t *SerialManager_GetCircularBufferFromPort(External_Port port);
void SerialManager_StartTransmission(External_Port port);
bool SubscribeToCommandI(KFly_Command command,
                         External_Port port,
                         uint16_t delay_ms);
bool SubscribeToCommand(KFly_Command command,
                        External_Port port,
                        uint16_t delay_ms);
bool UnsubscribeFromCommandI(KFly_Command command, External_Port port);
bool UnsubscribeFromCommand(KFly_Command command, External_Port port);
void UnsubscribeFromAllI(void);
void UnsubscribeFromAll(void);

#endif
