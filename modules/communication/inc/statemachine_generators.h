#ifndef __STATEMACHINE_GENERATORS_H
#define __STATEMACHINE_GENERATORS_H

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

bool GenerateMessage(KFly_Command command, External_Port port);
bool GenerateCustomMessage(KFly_Command command,
                           uint8_t *data,
                           uint16_t size,
                           External_Port port);
bool GenerateDebugMessage(uint8_t *data, 
                          uint32_t size, 
                          circular_buffer_t *Cbuff);

#endif
