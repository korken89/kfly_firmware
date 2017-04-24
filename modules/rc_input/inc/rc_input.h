#ifndef __RC_INPUT_H
#define __RC_INPUT_H

#include <kfly_defs.h>

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define RCINPUT_CAPTURE_TIMER_RATE      1000000
#define RCINPUT_CPPM_SYNC_LIMIT_MIN     3000    /* 3 ms */
#define RCINPUT_CPPM_SYNC_LIMIT_MAX     30000   /* 30 ms */
#define RCINPUT_RSSI_THRESHOLD_PERCENT  5       /* RSSI threshold in percent */
#define RCINPUT_RSSI_TIMEOUT            100     /* Number of bad RSSI
                                                   measurements to disable the
                                                   connection */
#define RCINPUT_NUMBER_OF_SWITCHES      3
#define RCINPUT_NO_CON_TIMEOUT_MS       500
#define RCINPUT_LOST_EVENTMASK          EVENT_MASK(0)
#define RCINPUT_ACTIVE_EVENTMASK        EVENT_MASK(1)
#define RCINPUT_NEWINPUT_EVENTMASK      EVENT_MASK(2)

#define RCINPUT_DATA_SIZE               (sizeof(rcinput_data_t))
#define RCINPUT_SETTINGS_SIZE           (sizeof(rcinput_settings_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Input capture channel selector.
 */
typedef enum {
    /**
     * @brief   Channel 1 selector.
     */
    RCINPUT_INPUT_CH1 = 0,
    /**
     * @brief   Channel 2 selector.
     */
    RCINPUT_INPUT_CH2 = 1,
    /**
     * @brief   Channel 3 selector.
     */
    RCINPUT_INPUT_CH3 = 2,
    /**
     * @brief   Channel 4 selector.
     */
    RCINPUT_INPUT_CH4 = 3,
    /**
     * @brief   Channel 5 selector.
     */
    RCINPUT_INPUT_CH5 = 4,
    /**
     * @brief   Channel 6 selector.
     */
    RCINPUT_INPUT_CH6 = 5,
    /**
     * @brief   Channel 7 selector.
     */
    RCINPUT_INPUT_CPPM = 6,
    /**
     * @brief   Channel 8 selector.
     */
    INPUT_RSSI = 7
} rcinput_capture_channel_t;

/**
 * @brief   Input capture type selector.
 */
typedef enum {
    /**
     * @brief   No mode.
     */
    RCINPUT_MODE_NONE = 0,
    /**
     * @brief   CPPM input: all channels on one input line.
     */
    RCINPUT_MODE_CPPM_INPUT = 1,
    /**
     * @brief   PWM input: all channels have its own input line.
     */
    RCINPUT_MODE_PWM_INPUT = 2
} rcinput_mode_selector_t;

/**
 * @brief   Input capture channel role selector.
 */
typedef enum PACKED_VAR {
    /**
     * @brief   Throttle role selector.
     */
    RCINPUT_ROLE_THROTTLE = 0,
    /**
     * @brief   Pitch role selector.
     */
    RCINPUT_ROLE_PITCH = 1,
    /**
     * @brief   Roll role selector.
     */
    RCINPUT_ROLE_ROLL = 2,
    /**
     * @brief   Yaw role selector.
     */
    RCINPUT_ROLE_YAW = 3,
    /**
     * @brief   Aux 1 role selector.
     */
    RCINPUT_ROLE_AUX1 = 4,
    /**
     * @brief   Aux 2 role selector.
     */
    RCINPUT_ROLE_AUX2 = 5,
    /**
     * @brief   Aux 3 role selector.
     */
    RCINPUT_ROLE_AUX3 = 6,

    /*
     * Switched bellow here!
     */

    /**
     * @brief   Role of a non-latching switch to arm the flight controller.
     */
    RCINPUT_ROLE_ARM_NONLATCH = 7,
    /**
     * @brief   Role for a latching switch to switch to remote serial control.
     */
    RCINPUT_ROLE_ENABLE_SERIAL_CONTROL = 8,
    /**
     * @brief   Flight mode selector.
     */
    RCINPUT_ROLE_FLIGHTMODE = 9,
    /**
     * @brief   The number of roles - 1. Always have at the end!
     */
    RCINPUT_ROLE_MAX = 10,
    /**
     * @brief   Start value for switches.
     */
    RCINPUT_ROLE_SWITCHES_START = RCINPUT_ROLE_ARM_NONLATCH,
    /**
     * @brief   Unused role selector.
     */
    RCINPUT_ROLE_OFF = 0xff
} rcinput_role_selector_t;

/**
 * @brief   Input role lookup definition.
 */
typedef struct {
    uint8_t index[RCINPUT_MAX_NUMBER_OF_INPUTS];
} rcinput_role_lookup_t;

/**
 * @brief   Input capture channel type.
 */
typedef enum PACKED_VAR {
    /**
     * @brief   Input type not set.
     */
    RCINPUT_TYPE_NONE = 0,
    /**
     * @brief   Analog type input.
     */
    RCINPUT_TYPE_ANALOG = 1,
    /**
     * @brief   Tri-state type input.
     */
    RCINPUT_TYPE_3_STATE = 2,
    /**
     * @brief   On/off type input.
     */
    RCINPUT_TYPE_ON_OFF = 3
} rcinput_type_selector_t;

/**
 * @brief   Input switch states.
 */
typedef enum PACKED_VAR {
    /**
     * @brief   Switch error, not a switch.
     */
    RCINPUT_SWITCH_UNDEFINED = 0,
    /**
     * @brief   Switch error, not a switch.
     */
    RCINPUT_SWITCH_NOT_SWITCH = 1,
    /**
     * @brief   Switch at bottom position.
     */
    RCINPUT_SWITCH_POSITION_BOTTOM = 2,
    /**
     * @brief   Switch at center position.
     */
    RCINPUT_SWITCH_POSITION_CENTER = 3,
    /**
     * @brief   Switch at top position.
     */
    RCINPUT_SWITCH_POSITION_TOP = 4
} rcinput_switch_position_t;

typedef struct PACKED_VAR {
    /**
     * @brief  All calibrated values.
     */
    float calibrated_value[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   State of the switches.
     */
    rcinput_switch_position_t switches[RCINPUT_NUMBER_OF_SWITCHES];
} rcinput_calibrated_data_t;

/**
 * @brief   Input capture data holder.
 */
typedef struct PACKED_VAR {
    /**
     * @brief   Calibrated and mapped data holder.
     */
    rcinput_calibrated_data_t calibrated_values;
    /**
     * @brief   Active connection indicator.
     */
    bool8_t active_connection;
    /**
     * @brief   Number of active channels.
     */
    uint16_t number_active_connections;
    /**
     * @brief   The value of each input channel.
     */
    uint16_t value[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Current RSSI value.
     */
    uint16_t rssi;
    /**
     * @brief   Current RSSI frequency.
     */
    uint16_t rssi_frequency;
} rcinput_data_t;

/**
 * @brief   Input capture settings holder.
 */
typedef struct PACKED_VAR {
    /**
     * @brief   Input capture top calibration value.
     */
    uint16_t ch_top[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input capture center calibration value.
     */
    uint16_t ch_center[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input capture bottom calibration value.
     */
    uint16_t ch_bottom[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input capture role selector.
     */
    rcinput_role_selector_t role[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input capture type selector.
     */
    rcinput_type_selector_t type[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input reverse selector.
     */
    bool8_t ch_reverse[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input capture mode selector.
     */
    rcinput_mode_selector_t mode;
    /**
     * @brief   RSSI usage selector.
     */
    bool8_t use_rssi;
} rcinput_settings_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void RCInputInit(void);
msg_t RCInputInitialization(void);
float RCInputGetInputLevel(rcinput_role_selector_t role);
rcinput_switch_position_t RCInputGetSwitchState(rcinput_role_selector_t role);
void vParseSetRCInputSettings(const uint8_t *payload,
                              const size_t data_length);
bool bActiveRCInputConnection(void);
rcinput_data_t *ptrGetRCInputData(void);
rcinput_settings_t *ptrGetRCInputSettings(void);
event_source_t *ptrGetRCInputEventSource(void);

#endif
