#ifndef __USB_ACCESS_H
#define __USB_ACCESS_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
extern mutex_t USB_write_lock;
#endif

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Claim the USB mutex.
 */
static inline void USBClaim(void)
{
#if (CH_CFG_USE_MUTEXES == TRUE)
    chMtxLock(&USB_write_lock);
#endif
}

/**
 * @brief   Release the USB mutex.
 */
static inline void USBRelease(void)
{
#if (CH_CFG_USE_MUTEXES == TRUE)
    chMtxUnlock(&USB_write_lock);
#endif
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void USBInit(void);
void USBDeinit(void);
void USBConnect(void);
void USBDisconnect(void);
bool isUSBActive(void);
void USBMutexInit(void);
size_t USBSendData(uint8_t *data, size_t size, systime_t timeout);
size_t USBReadByte(systime_t timeout);

#endif


