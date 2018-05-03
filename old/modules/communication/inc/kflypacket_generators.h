#ifndef __KFLYPACKET_GENERATORS_H
#define __KFLYPACKET_GENERATORS_H

#include "slip2kflypacket.h"

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

bool GenerateMessage(kfly_command_t command, external_port_t port);
bool GenerateCustomMessage(kfly_command_t command,
                           uint8_t *data,
                           uint16_t size,
                           external_port_t port);
bool GenerateDebugMessage(uint8_t *data,
                          uint32_t size,
                          circular_buffer_t *Cbuff);

#endif
