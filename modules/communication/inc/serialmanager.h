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

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vSerialManagerInit(void);
void vTaskUSBSerialManager(void *);
circular_buffer_t *SerialManager_GetCircularBufferFromPort(External_Port port);
void SerialManager_StartTransmission(External_Port port);
bool SubscribeToCommand(KFly_Command command,
                        External_Port port,
                        uint16_t delay_ms);
bool UnsubscribeFromCommand(KFly_Command command);
void UnsubscribeFromAll(void);

#endif
