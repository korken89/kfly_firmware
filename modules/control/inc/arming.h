#ifndef __ARMING_H
#define __ARMING_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Possible stick direction for arming the controllers.
 */
typedef enum PACKED_VAR
{
    /**
     * @brief   Arm direction not yet set.
     */
    ARMING_DIRECTION_STICK_NONE = 0,
    /**
     * @brief   Arm at pitch at min.
     */
    ARMING_DIRECTION_STICK_PITCH_MIN,
    /**
     * @brief   Arm at pitch at max.
     */
    ARMING_DIRECTION_STICK_PITCH_MAX,
    /**
     * @brief   Arm at roll at min.
     */
    ARMING_DIRECTION_STICK_ROLL_MIN,
    /**
     * @brief   Arm at roll at max.
     */
    ARMING_DIRECTION_STICK_ROLL_MAX,
    /**
     * @brief   Arm at yaw at min.
     */
    ARMING_DIRECTION_STICK_YAW_MIN,
    /**
     * @brief   Arm at yaw at max.
     */
    ARMING_DIRECTION_STICK_YAW_MAX,
    /**
     * @brief   Arm with non-latching switch.
     */
    ARMING_DIRECTION_NON_LATCHING_SWITCH
} arming_stick_direction_t;

/**
 * @brief   Possible stick regions for arming/disarming the controllers.
 */
typedef enum
{
    /**
     * @brief   Sticks are neither in the arm or disarm position.
     */
    ARMING_REGION_STICK_NO_REGION = 0,
    /**
     * @brief   Sticks are in the arm position.
     */
    ARMING_REGION_STICK_ARM,
    /**
     * @brief   Sticks are in the disarm position.
     */
    ARMING_REGION_STICK_DISARM,
    /**
     * @brief   The non-latching arm switch is in the correct region.
     */
    ARMING_REGION_NON_LATCHING_SWITCH_ACTIVE
} arming_stick_region_t;

/**
 * @brief   Settings for the arm and disarm functionality.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Stick threshold for the arm/disarm logic to react.
     */
    float stick_threshold;
    /**
     * @brief   Minimum throttle when armed (to spin propellers when armed).
     */
    float armed_min_throttle;
    /**
     * @brief   Stick direction to arm the controllers.
     */
    arming_stick_direction_t stick_direction;
    /**
     * @brief   Time (in 1/10th seconds) needed to hold the sticks or switches
     *          in correct position in order to arm the system.
     */
    uint8_t arm_stick_time;
    /**
     * @brief   Time (in seconds) needed to disarm the controllers if no
     *          throttle has been given.
     */
    uint8_t arm_zero_throttle_timeout;
} control_arm_settings_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void ArmingInit(void);
bool bIsSystemArmed(void);
float fGetArmedMinThrottle(void);
void vForceDisarm(const uint32_t key);
control_arm_settings_t *ptrGetControlArmSettings(void);

#endif
