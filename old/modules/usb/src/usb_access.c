#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "usb_access.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/**
 * @brief       USB access mutex.
 */
#if (CH_CFG_USE_MUTEXES == TRUE) || defined(__DOXYGEN__)
mutex_t USB_write_lock;
#endif

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief       Initialize the USB.
 */
void USBInit(void)
{
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbStart(serusbcfg.usbp, &usbcfg);
}

/**
 * @brief       De-initialize the USB.
 */
void USBDeinit(void)
{
    usbStop(serusbcfg.usbp);
    sduStop(&SDU1);
}

/**
 * @brief       Connect the bus (enable the pull-up).
 */
void USBConnect(void)
{
    usbConnectBus(serusbcfg.usbp);
}

/**
 * @brief       Disconnect the bus (disable the pull-up).
 */
void USBDisconnect(void)
{
    usbDisconnectBus(serusbcfg.usbp);
}

/**
 * @brief       Check if there is an active USB connection.
 * 
 * @return      true if there is an active connection, else false.
 */
bool isUSBActive(void)
{
    if (serusbcfg.usbp->state == USB_ACTIVE)
        return true;
    else
        return false;
}

/**
 * @brief       Initialized the USB access mutex.
 */
void USBMutexInit(void)
{
#if (CH_CFG_USE_MUTEXES == TRUE)
    chMtxObjectInit(&USB_write_lock);
#endif
}

/**
 * @brief              Send data over the USB with timeout.
 * 
 * @param[in] data     Pointer to the data.
 * @param[in] size     Number of bytes to send.
 * @param[in] timeout  Timeout for the transmission.
 * @return             The number of bytes sent.
 */
size_t USBSendData(uint8_t *data, size_t size, systime_t timeout)
{
    size_t sent;

    sent = chnWriteTimeout(&SDU1, data, size, timeout);

    return sent;
}

/**
 * @brief              Receive data over the USB with timeout.
 * 
 * @param[in] timeout  Timeout for the reception.
 * @return             The byte read.
 *
 * @note               The USB must be active for the timeout to work. If there
 *                     is no connection it will return directly. 
 */
size_t USBReadByte(systime_t timeout)
{
    return chnGetTimeout(&SDU1, timeout);
}