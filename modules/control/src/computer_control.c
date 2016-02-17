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
volatile bool control_init;
volatile reference_source_t reference_source;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void vt_disarm_callback(void *p)
{
    (void)p;

    osalSysLockFromISR();

    /* No new computer control message, switch to manual mode. */
    reference_source = REFERENCE_SOURCE_MANUAL;
    chVTResetI(&vt_disarm);

    osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

void ComputerControlInit(void)
{
    control_init = true;
    reference_source = REFERENCE_SOURCE_MANUAL;
    chVTObjectInit(&vt_disarm);
}

reference_source_t GetReferenceSource(void)
{
    return reference_source;
}

flightmode_t GetComputerFlightMode(void)
{
    return computer_control.mode;
}

void vParseComputerControlPacket(const uint8_t *payload, const uint8_t size)
{
    if (size == COMPUTER_CONTROL_MESSAGE_SIZE)
    {
        osalSysLock();

        memcpy(&computer_control,
               payload,
               COMPUTER_CONTROL_MESSAGE_SIZE);

        /* Set the reference source to computer control. */
        reference_source = REFERENCE_SOURCE_COMPUTER_CONTROL;

        /* Timeout for no new messags set to 500 ms. */
        chVTSetI(&vt_disarm, MS2ST(500), vt_disarm_callback, NULL);

        osalSysUnlock();
    }
}
