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

/**
 * @brief       Callback for communication timeout, will reset the reference
 *              source to manual control.
 *
 * @param[in] p Rate limits around roll (x), pitch (y) and yaw (z).
 */
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

/**
 * @brief   Initialization of the computer control.
 */
void ComputerControlInit(void)
{
    control_init = true;
    reference_source = REFERENCE_SOURCE_MANUAL;
    chVTObjectInit(&vt_disarm);
}

/**
 * @brief   Returns the current reference source.
 *
 * @return  The current reference source.
 */
reference_source_t GetReferenceSource(void)
{
    return reference_source;
}

/**
 * @brief   Returns the current flight mode received from the computer control.
 *
 * @return  Return the flight mode of the computer control.
 */
flightmode_t GetComputerFlightMode(void)
{
    return computer_control.mode;
}

/**
 * @brief   Gets the current attitude and trottle reference from the computer
 *          control.
 *
 * @param[out] attitude_ref     Where the attitude reference will be saved.
 * @param[out] throttle_ref     Where the throttle reference will be saved.
 */
void GetComputerAttitudeReference(quaternion_t *attitude_ref,
                                  float *throttle_ref)
{
    *attitude_ref = computer_control.attitude.attitude;
    *throttle_ref = computer_control.attitude.throttle;
}

/**
 * @brief   Gets the current rate and trottle reference from the computer
 *          control.
 *
 * @param[out] rate_ref         Where the rate reference will be saved.
 * @param[out] throttle_ref     Where the throttle reference will be saved.
 */
void GetComputerRateReference(vector3f_t *rate_ref,
                              float *throttle_ref)
{
    *rate_ref = computer_control.rate.rate;
    *throttle_ref = computer_control.rate.throttle;
}

/**
 * @brief   Gets the current normalized torque and trottle reference from the
 *          computer control.
 *
 * @param[out] torque_ref       Where the normalized torque will be saved.
 * @param[out] throttle_ref     Where the throttle reference will be saved.
 */
void GetComputerIndirectReference(vector3f_t *torque_ref,
                                  float *throttle_ref)
{
    *torque_ref = computer_control.indirect_control.torque;
    *throttle_ref = computer_control.indirect_control.throttle;
}

/**
 * @brief   Gets the current direct motor commands.
 *
 * @param[out] output   Where the direct motor commands will be saved.
 */
void GetComputerDirectReference(float output[8])
{
    int i;

    for (i = 0; i < 8; i++)
        output[i] = ((float)computer_control.direct_control[i]) *
            (1.0f / 65535.0f);
}

/**
 * @brief       Parses a computer control message.
 *
 * @param[in] payload   Byte array with payload.
 * @param[in] size      Size of the payload.
 */
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
