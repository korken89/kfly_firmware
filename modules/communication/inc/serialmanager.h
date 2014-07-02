#ifndef __SERIALMANAGER_H
#define __SERIALMANAGER_H

#include "statemachine.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

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
static inline bool SubscribeToCommand(KFly_Command command,
                                      External_Port port,
                                      uint16_t delay_ms)
{
    osalSysLock();
    SubscribeToCommandI(command, port, delay_ms);
    osalSysUnlock();
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
static inline bool UnsubscribeFromCommand(KFly_Command command,
                                          External_Port port)
{
    osalSysLock();
    UnsubscribeFromCommandI(command, port);
    osalSysUnlock();
}

/**
 * @brief   Removes all ongoing subscriptions.
 */
static inline void UnsubscribeFromAll(void)
{
    osalSysLock();
    UnsubscribeFromAllI();
    osalSysUnlock();
}

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
bool UnsubscribeFromCommandI(KFly_Command command, External_Port port);
void UnsubscribeFromAllI(void);

#endif
