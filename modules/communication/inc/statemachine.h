#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

#include "statemachine_types.h"

/* Defines */

/* Typedefs */

/* Global variable defines */

/* Global function defines */
void vInitStatemachineDataHolder(parser_holder_t *pHolder,
                                 Port_Type port,
                                 uint8_t *buffer);
void vStatemachineDataEntry(uint8_t data, parser_holder_t *pHolder);
void vWaitingForSYNC(uint8_t data, parser_holder_t *pHolder);
void vWaitingForSYNCorCMD(uint8_t data, parser_holder_t *pHolder);
void vRxCmd(uint8_t data, parser_holder_t *pHolder);
void vRxSize(uint8_t data, parser_holder_t *pHolder);
void vRxCRC8(uint8_t data, parser_holder_t *pHolder);
void vRxData(uint8_t data, parser_holder_t *pHolder);
void vRxCRC16_1(uint8_t data, parser_holder_t *pHolder);
void vRxCRC16_2(uint8_t data, parser_holder_t *pHolder);
void vReturnACK(parser_holder_t *pHolder);

#endif
