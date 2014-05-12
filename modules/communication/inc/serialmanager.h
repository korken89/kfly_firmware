#ifndef __SERIALMANAGER_H
#define __SERIALMANAGER_H

/* Defines */
#define tskSerialManagerPRIORITY        (tskIDLE_PRIORITY + 1)
#define SERIAL_BUFFER_SIZE				256

/* Typedefs */

/* Global variable defines */

/* Global function defines */
void vSerialManagerInit(void);
void vTaskUSBSerialManager(void *);

#endif
