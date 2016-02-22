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
    STICK_NONE = 0,
    /**
     * @brief   Arm at pitch at min.
     */
    STICK_PITCH_MIN,
    /**
     * @brief   Arm at pitch at max.
     */
    STICK_PITCH_MAX,
    /**
     * @brief   Arm at roll at min.
     */
    STICK_ROLL_MIN,
    /**
     * @brief   Arm at roll at max.
     */
    STICK_ROLL_MAX,
    /**
     * @brief   Arm at yaw at min.
     */
    STICK_YAW_MIN,
    /**
     * @brief   Arm at yaw at max.
     */
    STICK_YAW_MAX
} arming_stick_direction_t;

/**
 * @brief   Possible stick regions for arming/disarming the controllers.
 */
typedef enum
{
    /**
     * @brief   Sticks are neither in the arm or disarm position.
     */
    STICK_NO_REGION = 0,
    /**
     * @brief   Sticks are in the arm position.
     */
    STICK_ARM_REGION,
    /**
     * @brief   Sticks are in the disarm position.
     */
    STICK_DISARM_REGION
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
     * @brief   Time (in seconds) needed to hold the sticks in correct
     *          position in order to arm the system.
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
float fGetDisarmedThrottle(void);
void vForceDisarm(const uint32_t key);
control_arm_settings_t *ptrGetControlArmSettings(void);

#endif
