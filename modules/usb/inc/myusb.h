#ifndef __MYUSB_H
#define __MYUSB_H

/* Defines */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/* Typedefs */

/* Macros */

/* Global variables */
extern SerialUSBDriver SDU1;
extern const USBConfig usbcfg;
extern const SerialUSBConfig serusbcfg;

/* Global functions */
bool_t isUSBActive(void);

#endif
