/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "computer_control.h"
#include <string.h>

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

computer_control_reference_t computer_control;
virtual_timer_t vt_disarm;
volatile bool bControlInit = true;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void vt_disarm_callback(void *p)
{
    (void)p;

    osalSysLockFromISR();

    //computer_control.throttle = 0;
    chVTResetI(&vt_disarm);

    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void vParseComputerControlPacket(const uint8_t *payload, const uint8_t size)
{
    if (size == COMPUTER_CONTROL_MESSAGE_SIZE)
    {
        if (bControlInit == true)
        {
            chVTObjectInit(&vt_disarm);
            bControlInit = false;
        }
        else
        {
            osalSysLock();

            memcpy(&computer_control,
                   payload,
                   COMPUTER_CONTROL_MESSAGE_SIZE);

            /* TODO: Check the payload in case of errors. */

            chVTSetI(&vt_disarm, MS2ST(500), vt_disarm_callback, NULL);

            osalSysUnlock();
        }
    }
}
