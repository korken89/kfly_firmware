/* *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "arming.h"
#include "computer_control.h"
#include "rc_input.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define ARM_RATE                20 /* Hz */

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
motor_override_t override_settings;
THD_WORKING_AREA(waThreadControlArming, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

static void CheckMotorOverride(void)
{
  if (!force_disarm)
  {
    if (override_settings.active &&
        (override_settings.timeout > ARM_RATE / 2)) /* Half second timeout */
      override_settings.active = false;
    else
      override_settings.timeout++;
  }
  else
    override_settings.active = false;
}

static void CheckArmingConditions(void)
{
    static uint16_t arm_time = 0, disarm_time = 0, timeout_time = 0;
    static bool latch_released = false;
    static float level;
    static arming_stick_region_t current_region;

    /* Check emergency stop. */
    if (force_disarm)
    {
        system_armed = false;
        force_disarm = false;
        latch_released = false;
        arm_time = 0;
        disarm_time = 0;
        timeout_time = 0;

        return;
    }

    /* Check so there is an active RC connection and the arming stick
       position has been set */
    if ((bActiveRCInputConnection() == true) &&
        (arm_settings.stick_direction != ARMING_DIRECTION_STICK_NONE))
    {
        /* Check so the sticks are in the correct region ( */
        current_region = SticksInRegion();


        /* If the computer control is active, don't disarm based on time. */
        if ((system_armed == true) && (ComputerControlEnabled() == true))
        {
            timeout_time = 0;
        }

        if (arm_settings.stick_direction == ARMING_DIRECTION_NON_LATCHING_SWITCH)
        {
            /*
             * Switch based arming and disarm.
             */
            if (current_region == ARMING_REGION_NON_LATCHING_SWITCH_ACTIVE)
            {
                level = RCInputGetInputLevel(RCINPUT_ROLE_THROTTLE);

                /* If it is in the correct state, increase the arm counter,
                 * and if it has reached the correct time - arm. */
                if ((system_armed == false) && (latch_released == true) &&
                    (level <= arm_settings.stick_threshold))
                {
                    if ((arm_time / ARM_RATE * 10) >= arm_settings.arm_stick_time)
                    {
                        system_armed = true;
                        latch_released = false;
                    }
                    else
                    {
                        arm_time++;
                        timeout_time = 0;
                    }
                }
                else if ((system_armed == true) &&
                         (latch_released == true))
                {
                    /* Disarm does not need to use the wait time as arm
                     * does, only a quick tap will disarm. */
                    latch_released = false;
                    system_armed = false;
                    arm_time = 0;
                    timeout_time = 0;
                }
            }
            else
            {
                arm_time = 0;
                timeout_time++;
                latch_released = true;
            }
        }
        else
        {
            /*
             * Stick based arming and disarm.
             */

            if (current_region == ARMING_REGION_STICK_ARM)
            {
                /* Check if the required time has been reached
                   to arm the system */
                if ((arm_time / ARM_RATE * 10) >= arm_settings.arm_stick_time)
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
            else if (current_region == ARMING_REGION_STICK_DISARM)
            {
                /* Check if the required time has been reached
                   to disarm the system*/
                if ((disarm_time / ARM_RATE) >= arm_settings.arm_stick_time)
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

            }
        }

        /* Timeout counter. */
        /* Check the zero throttle timeout */
        if ((arm_settings.arm_zero_throttle_timeout != 0) &&
            (system_armed == true))
        {
            if ((RCInputGetInputLevel(RCINPUT_ROLE_THROTTLE) <=
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
    else
    {
        system_armed = false;
    }
}


/**
 * @brief           Thread for the arm and disarm functionality.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadControlArming, arg)
{
    (void)arg;

    override_settings.timeout = 0;

    /* Set thread name */
    chRegSetThreadName("Arm Control");

    while (1)
    {
        /* Check the conditions for arming and disarming */
        osalThreadSleepMilliseconds(1000 / ARM_RATE);

        /*
         * Check all conditions for arming and disarming the system
         */
        if (!override_settings.active)
            CheckArmingConditions();

        /* Check for override */
        CheckMotorOverride();
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
    rcinput_role_selector_t sel;
    bool is_min;
    float level, threshold;

    threshold = arm_settings.stick_threshold;

    if (arm_settings.stick_direction == ARMING_DIRECTION_NON_LATCHING_SWITCH)
    {
        sel = RCINPUT_ROLE_ARM_NONLATCH;

        if  (RCInputGetSwitchState(sel) == RCINPUT_SWITCH_POSITION_TOP)
            return ARMING_REGION_NON_LATCHING_SWITCH_ACTIVE;
    }
    else
    {
        level = RCInputGetInputLevel(RCINPUT_ROLE_THROTTLE);

        /* Check so the throttle is within the threshold */
        if (level <= threshold)
        {
            /* Determine which role the arm is linked to and if it is min/max. */
            switch (arm_settings.stick_direction)
            {

                case ARMING_DIRECTION_STICK_PITCH_MIN:
                    sel = RCINPUT_ROLE_PITCH;
                    is_min = true;
                    break;

                case ARMING_DIRECTION_STICK_PITCH_MAX:
                    sel = RCINPUT_ROLE_PITCH;
                    is_min = false;
                    break;

                case ARMING_DIRECTION_STICK_ROLL_MIN:
                    sel = RCINPUT_ROLE_ROLL;
                    is_min = true;
                    break;

                case ARMING_DIRECTION_STICK_ROLL_MAX:
                    sel = RCINPUT_ROLE_ROLL;
                    is_min = false;
                    break;

                case ARMING_DIRECTION_STICK_YAW_MIN:
                    sel = RCINPUT_ROLE_YAW;
                    is_min = true;
                    break;

                case ARMING_DIRECTION_STICK_YAW_MAX:
                    sel = RCINPUT_ROLE_YAW;
                    is_min = false;
                    break;

                default:
                    return ARMING_REGION_STICK_NO_REGION;

            }

            level = RCInputGetInputLevel(sel);

            /* Calculate the threshold value. The *2 comes from the fact that
               the throttle has half the span of the other sticks so double the
               threshold is needed to the same relative threshold. */
            threshold = 1.0f - 2.0f * threshold;

            if (is_min == true)
            {
                if (level <= -threshold)
                    return ARMING_REGION_STICK_ARM;
                else if (level >= threshold)
                    return ARMING_REGION_STICK_DISARM;
            }
            else
            {
                if (level >= threshold)
                    return ARMING_REGION_STICK_ARM;
                else if (level <= -threshold)
                    return ARMING_REGION_STICK_DISARM;
            }
        }
    }

    return ARMING_REGION_STICK_NO_REGION;
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
    arm_settings.stick_direction = ARMING_DIRECTION_STICK_NONE;
    arm_settings.arm_stick_time = 10;
    arm_settings.arm_zero_throttle_timeout = 30;

    /* Initialize override */
    override_settings.active = false;

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
    return system_armed || override_settings.active;
}

/**
 * @brief       Returns the minimum throttle to rotate the propellers when
 *              disarmed.
 *
 * @return      The minimum throttle value.
 */
float fGetArmedMinThrottle(void)
{
    return arm_settings.armed_min_throttle;
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

/**
 * @brief       Parses a motor override command.
 *
 * @param[in] data  Pointer to the data message.
 * @param[in] size  Size of the data message.
 */
void vParseMotorOverride(const uint8_t* data, const uint8_t size)
{
  if (size == sizeof(float)*8)
  {
    bool sane_values = true;

    memcpy(override_settings.values, data, size);

    /* Sanity check */
    for (int i = 0; i < 8; i++)
      if ((override_settings.values[i] < 0.0f) ||
          (override_settings.values[i] > 1.0f))
        sane_values = false;

    if (sane_values)
    {
      override_settings.timeout = 0;
      override_settings.active = true;
    }
    else
      override_settings.active = false;
  }
}

/**
 * @brief     Checks if the motor override is active.
 *
 * @return    Returns true if the motor override is active.
 */
bool bMotorOverrideActive(void)
{
  return override_settings.active;
}

/**
 * @brief     Saves the motor override values.
 *
 * @param[out] dest   Pointer to where the motor override shall be saved.
 */
void vGetMotorOverrideValues(float dest[8])
{
  osalSysLock();

  for (int i = 0; i < 8; i++)
    dest[i] = override_settings.values[i];

  osalSysUnlock();
}

