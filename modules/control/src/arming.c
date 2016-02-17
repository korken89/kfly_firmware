/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "arming.h"
#include "rc_input.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define ARM_RATE                10 /* Hz */

static arming_stick_region_t SticksInRegion(void);

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

static volatile bool system_armed;
static volatile bool force_disarm;
control_arm_settings_t arm_settings;
THD_WORKING_AREA(waThreadControlArming, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief           Thread for the arm and disarm functionality.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadControlArming, arg)
{
    (void)arg;

    uint16_t arm_time = 0, disarm_time = 0, timeout_time = 0;
    arming_stick_region_t current_region;

    /* Set thread name */
    chRegSetThreadName("Arm Control");

    while (1)
    {
        /* Check the conditions for arming and disarming */
        osalThreadSleepMilliseconds(1000 / ARM_RATE);

        if (system_armed == true)
            palSetPad(GPIOC, GPIOC_LED_ERR);
        else
            palClearPad(GPIOC, GPIOC_LED_ERR);

        /*
         * Check all conditions for arming and disarming the system
         */

        /* TODO: Add support for switch arming. */

        /* Check so there is an active RC connection and st the arming stick
           position has been set */
        if ((bActiveRCInputConnection() == true) &&
            (arm_settings.stick_direction != STICK_NONE))
        {
            /* Check emergency stop, TODO: Change to switch arming. */
            if ((RCInputGetInputLevel(ROLE_AUX1) < 0.5f) || force_disarm)
            {
                system_armed = false;
                force_disarm = false;
                arm_time = 0;
                disarm_time = 0;
                timeout_time = 0;
            }
            else
            {
                /* Check so the sticks are in the correct region */
                current_region = SticksInRegion();

                if (current_region == STICK_ARM_REGION)
                {
                    /* Check if the required time has been reached
                       to arm the system */
                    if ((arm_time / ARM_RATE) > arm_settings.arm_stick_time)
                    {
                        system_armed = true;
                    }
                    else
                    {
                        arm_time++;
                        disarm_time = 0;
                        timeout_time = 0;
                    }
                }
                else if (current_region == STICK_DISARM_REGION)
                {
                    /* Check if the required time has been reached
                       to disarm the system*/
                    if ((disarm_time / ARM_RATE) > arm_settings.arm_stick_time)
                    {
                        system_armed = false;
                    }
                    else
                    {
                        disarm_time++;
                        arm_time = 0;
                        timeout_time = 0;
                    }
                }
                else
                {
                    /* Sticks are not in the correct region,
                       reset timing counters */
                    arm_time = 0;
                    disarm_time = 0;

                    /* Check the zero throttle timeout */
                    if (arm_settings.arm_zero_throttle_timeout != 0)
                    {
                        if ((RCInputGetInputLevel(ROLE_THROTTLE) <=
                            arm_settings.stick_threshold))
                        {
                            /* Check if the required time has passed to disarm due
                               to timeout, else increment the timing counter */
                            if ((timeout_time / ARM_RATE) >
                                arm_settings.arm_zero_throttle_timeout)
                            {
                                system_armed = false;
                            }
                            else
                            {
                                timeout_time++;
                                arm_time = 0;
                                disarm_time = 0;
                            }
                        }
                        /* The throttle is not in the correct position,
                           reset the timing counter */
                        else
                            timeout_time = 0;
                    }
                }
            }

        }
        else
        {
            system_armed = false;
        }
    }
}

/**
 * @brief           Checks if the sticks are in the correct position for Arm
 *                  Disarm access and returns the current region of the sticks.
 *
 * @return          Returns the current region the sticks are in.
 */
static arming_stick_region_t SticksInRegion(void)
{
    input_role_selector_t sel;
    bool is_min;
    float level, threshold;

    threshold = arm_settings.stick_threshold;
    level = RCInputGetInputLevel(ROLE_THROTTLE);

    /* Check so the throttle is within the threshold */
    if (level <= threshold)
    {
        /* Determine which role the arm is linked to and if it is min/max. */
        switch (arm_settings.stick_direction)
        {
            case STICK_PITCH_MIN:
                sel = ROLE_PITCH;
                is_min = true;
                break;

            case STICK_PITCH_MAX:
                sel = ROLE_PITCH;
                is_min = false;
                break;

            case STICK_ROLL_MIN:
                sel = ROLE_ROLL;
                is_min = true;
                break;

            case STICK_ROLL_MAX:
                sel = ROLE_ROLL;
                is_min = false;
                break;

            case STICK_YAW_MIN:
                sel = ROLE_YAW;
                is_min = true;
                break;

            case STICK_YAW_MAX:
                sel = ROLE_YAW;
                is_min = false;
                break;

            default:
                return STICK_NO_REGION;

        }

        /* Calculate the threshold value. The *2 comes from the fact that the
           throttle has half the span of the other sticks so double the
           threshold is needed to the same relative threshold. */
        threshold = 1.0f - 2.0f * threshold;

        /* Check so the last role is within the threshold */
        level = RCInputGetInputLevel(sel);

        if (is_min == true)
        {
            if (level <= -threshold)
                return STICK_ARM_REGION;
            else if (level >= threshold)
                return STICK_DISARM_REGION;
        }
        else
        {
            if (level >= threshold)
                return STICK_ARM_REGION;
            else if (level <= -threshold)
                return STICK_DISARM_REGION;
        }
    }

    return STICK_NO_REGION;
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief           Initializes the arming system.
 */
void ArmingInit(void)
{
    /* Set starting state. */
    system_armed = false;
    force_disarm = false;

    /* Initialize the arming structures */
    arm_settings.stick_threshold = 0.0f;
    arm_settings.armed_min_throttle = 0.0f;
    arm_settings.stick_direction = STICK_NONE;
    arm_settings.arm_stick_time = 5;
    arm_settings.arm_zero_throttle_timeout = 30;

    /* Initialize arming control thread */
    chThdCreateStatic(waThreadControlArming,
                      sizeof(waThreadControlArming),
                      HIGHPRIO - 1,
                      ThreadControlArming,
                      NULL);
}

/**
 * @brief       Returns if the system is armed or not.
 *
 * @return      The current state of the arming.
 */
bool bIsSystemArmed(void)
{
    return system_armed;
}

/**
 * @brief       Forces the system to disarm if the correct key has been
 *              received. Key is 0xdeadbeef
 */
void vForceDisarm(const uint32_t key)
{
    if (key == 0xdeadbeef)
        force_disarm = true;
}

/**
 * @brief       Return the pointer to the controller arm structure.
 *
 * @return      Pointer to the controller arm structure.
 */
control_arm_settings_t *ptrGetControlArmSettings(void)
{
    return &arm_settings;
}

