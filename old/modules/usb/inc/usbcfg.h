#ifndef __USBCFG_H
#define __USBCFG_H

/* Defines */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/* Typedefs */

/* Global variables */
extern const USBConfig usbcfg;
extern SerialUSBDriver SDU1;
extern const SerialUSBConfig serusbcfg;

#endif
