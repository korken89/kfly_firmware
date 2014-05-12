#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

/* Defines */

/* Typedefs */

/* Global variable defines */

/* Global function defines */
void vStatemachineDataEntry(uint8_t, Parser_Holder_Type *);
void vWaitingForSYNC(uint8_t, Parser_Holder_Type *);
void vWaitingForSYNCorCMD(uint8_t, Parser_Holder_Type *);
void vRxCmd(uint8_t, Parser_Holder_Type *);
void vRxSize(uint8_t, Parser_Holder_Type *);
void vRxCRC8(uint8_t, Parser_Holder_Type *);
void vRxData(uint8_t, Parser_Holder_Type *);
void vRxCRC16_1(uint8_t, Parser_Holder_Type *);
void vRxCRC16_2(uint8_t, Parser_Holder_Type *);
void vReturnACK(Parser_Holder_Type *);

#endif
