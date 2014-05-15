#ifndef __SERIALMANAGER_TYPES_H
#define __SERIALMANAGER_TYPES_H

#include "circularbuffer.h"

typedef enum
{   /* This will help the parser function to identity the port receiving data */
    PORT_USB = 0,
    PORT_AUX1,
    PORT_AUX2,
    PORT_AUX3,
    PORT_AUX4       /* CAN port */
} Port_Type;

typedef union
{
    uint8_t data[4];
    float value;
} f2bArray;

typedef union
{
    uint8_t data[2];
    int16_t value;
} sint2bArray;

typedef union
{
    uint8_t data[4];
    int32_t value;
} int2bArray;

typedef struct
{
    thread_t *ptrUSBDataPump;
    Circular_Buffer_Type USBTransmitBuffer;

    thread_t *ptrAUX1DataPump;
    Circular_Buffer_Type AUX1TransmitBuffer;

    thread_t *ptrAUX2DataPump;
    Circular_Buffer_Type AUX2TransmitBuffer;

    thread_t *ptrAUX3DataPump;
    Circular_Buffer_Type AUX3TransmitBuffer;
    
    thread_t *ptrAUX4DataPump;
    Circular_Buffer_Type AUX4TransmitBuffer;
} Serial_Datapump_Holder;

#endif
