#ifndef __MYUSB_H
#define __MYUSB_H

/* Defines */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/* Typedefs */

/* Macros */

/* Global variables */
extern mutex_t USB_write_lock;
extern SerialUSBDriver SDU1;
extern const USBConfig usbcfg;
extern const SerialUSBConfig serusbcfg;

/* Inline functions */
static inline void ClaimUSB(void)
{
  chMtxLock(&USB_write_lock);
}

static inline void ReleaseUSB(void)
{
  chMtxUnlock(&USB_write_lock);
}

/* Global functions */
bool isUSBActive(void);
void vUSBMutexInit(void);
bool bUSBSendData(uint8_t *data, uint32_t size);

#endif
