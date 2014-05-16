#ifndef __SERIALMANAGER_H
#define __SERIALMANAGER_H

#include "serialmanager_types.h"

/* Defines */
#define tskSerialManagerPRIORITY        (tskIDLE_PRIORITY + 1)
#define START_TRANSMISSION_EVENT		((eventmask_t)1)

/* Typedefs */

/* Global variable defines */

/* Global function defines */
void vSerialManagerInit(void);
void vTaskUSBSerialManager(void *);
Circular_Buffer_Type *SerialManager_GetCircularBufferFromPort(Port_Type port);
void SerialManager_StartTransmission(Port_Type port);
bool SerialManager_USBTransmitCircularBuffer(Circular_Buffer_Type *Cbuff);

#endif
