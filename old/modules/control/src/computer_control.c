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
volatile bool computer_control_active;

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
    computer_control_active = false;
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
    computer_control_active = false;

    chVTObjectInit(&vt_disarm);
}

/**
 * @brief   Returns the current reference source.
 *
 * @return  The current reference source.
 */
bool ComputerControlLinkActive(void)
{
    return computer_control_active;
}

bool ComputerControlEnabled(void)
{
    return ((bIsSystemArmed() == true) &&
            (ComputerControlLinkActive() == true) &&
            (RCInputGetSwitchState(RCINPUT_ROLE_ENABLE_SERIAL_CONTROL) ==
                RCINPUT_SWITCH_POSITION_TOP));
}

/**
 * @brief   Returns the current flight mode received from the computer control.
 *
 * @return  Return the flight mode of the computer control.
 */
flightmode_t ComputerControlGetFlightmode(void)
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
void ComputerControlGetAttitudeReference(quaternion_t *attitude_ref,
                                         float *throttle_ref)
{
    *attitude_ref = computer_control.attitude.attitude;
    *throttle_ref = computer_control.attitude.throttle;
}

void ComputerControlGetAttitudeEulerReference(vector3f_t *euler_ref,
                                              float *throttle_ref)
{
    // Get the roll angle, pitch angle, and yaw rate
    *euler_ref = *((vector3f_t *)&computer_control.attitude_euler.roll);
    *throttle_ref = computer_control.attitude_euler.throttle;
}

/**
 * @brief   Gets the current rate and trottle reference from the computer
 *          control.
 *
 * @param[out] rate_ref         Where the rate reference will be saved.
 * @param[out] throttle_ref     Where the throttle reference will be saved.
 */
void ComputerControlGetRateReference(vector3f_t *rate_ref,
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
void ComputerControlGetIndirectReference(vector3f_t *torque_ref,
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
void ComputerControlGetDirectReference(float output[8])
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
        computer_control_active = true;

        /* Timeout for no new messags set to 500 ms. */
        chVTSetI(&vt_disarm, MS2ST(500), vt_disarm_callback, NULL);

        osalSysUnlock();
    }
}
