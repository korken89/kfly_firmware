#ifndef __SERIALMANAGER_TYPES_H
#define __SERIALMANAGER_TYPES_H

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

#endif
