/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "position_loop.h"
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

position_control_package_t position_control_package;
virtual_timer_t vt_disarm;
volatile bool bControlInit = true;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void vt_disarm_callback(void *p)
{
    (void)p;

    osalSysLockFromISR();

    position_control_package.throttle = 0;
    chVTResetI(&vt_disarm);

    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

float fGetComputerControlThrottle(void)
{
    return ((float)position_control_package.throttle) / 1000.0f;
}

float fGetComputerControlRoll(void)
{
    return ((float)position_control_package.roll) / 1000.0f;
}

float fGetComputerControlPitch(void)
{
    return ((float)position_control_package.pitch) / 1000.0f;
}

float fGetComputerControlYaw(void)
{
    return ((float)position_control_package.yaw) / 1000.0f;
}

void vParseComputerControlPackage(uint8_t *payload, uint8_t size)
{
    if (bControlInit == true)
    {
        chVTObjectInit(&vt_disarm);
        bControlInit = false;
    }
    else
    {
        if (size == POSITION_CONTROL_MESSAGE_SIZE)
        {
            osalSysLock();

            memcpy(&position_control_package,
                   payload,
                   POSITION_CONTROL_MESSAGE_SIZE);

            chVTSetI(&vt_disarm, MS2ST(500), vt_disarm_callback, NULL);

            osalSysUnlock();
        }
    }
}
