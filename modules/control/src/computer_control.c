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

computer_control_t computer_control;
virtual_timer_t vt_disarm;
volatile bool bControlInit = true;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void vt_disarm_callback(void *p)
{
    (void)p;

    osalSysLockFromISR();

    computer_control.throttle = 0;
    chVTResetI(&vt_disarm);

    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

float fGetComputerControlThrottle(void)
{
    return ((float)computer_control.throttle) / 1000.0f;
}

float fGetComputerControlRoll(void)
{
    return ((float)computer_control.roll) / 1000.0f;
}

float fGetComputerControlPitch(void)
{
    return ((float)computer_control.pitch) / 1000.0f;
}

float fGetComputerControlYaw(void)
{
    return ((float)computer_control.yaw) / 1000.0f;
}

void vParseComputerControlPackage(const uint8_t *payload, const uint8_t size)
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

            chVTSetI(&vt_disarm, MS2ST(500), vt_disarm_callback, NULL);

            osalSysUnlock();
        }
    }
}
