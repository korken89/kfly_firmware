#ifndef __RC_INPUT_H
#define __RC_INPUT_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define RCINPUT_CAPTURE_TIMER_RATE      1000000
#define RCINPUT_MAX_NUMBER_OF_INPUTS    12
#define RCINPUT_CPPM_SYNC_LIMIT_MIN     3000    /* 3 ms */
#define RCINPUT_CPPM_SYNC_LIMIT_MAX     30000   /* 30 ms */
#define RCINPUT_RSSI_THRESHOLD_PERCENT  5       /* RSSI threshold in percent */
#define RCINPUT_RSSI_TIMEOUT            100     /* Number of bad RSSI
                                                   measurements to disable the 
                                                   connection */
#define RCINPUT_NO_CON_TIMEOUT_MS       500
#define RCINPUT_LOST_EVENTMASK          EVENT_MASK(0)
#define RCINPUT_ACTIVE_EVENTMASK        EVENT_MASK(1)
#define RCINPUT_NEWINPUT_EVENTMASK      EVENT_MASK(2)
#define RCINPUT_ROLE_TO_INDEX_BITS      4
#define RCINPUT_ROLE_TO_INDEX_MASK      0x0f

#define RCINPUT_DATA_SIZE               (sizeof(RCInput_Data))
#define RCINPUT_SETTINGS_SIZE           (sizeof(RCInput_Settings))

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
    INPUT_CH1 = 0,
    /**
     * @brief   Channel 2 selector.
     */
    INPUT_CH2 = 1,
    /**
     * @brief   Channel 3 selector.
     */
    INPUT_CH3 = 2,
    /**
     * @brief   Channel 4 selector.
     */
    INPUT_CH4 = 3,
    /**
     * @brief   Channel 5 selector.
     */
    INPUT_CH5 = 4,
    /**
     * @brief   Channel 6 selector.
     */
    INPUT_CH6 = 5,
    /**
     * @brief   Channel 7 selector.
     */
    INPUT_CPPM = 6,
    /**
     * @brief   Channel 8 selector.
     */
    INPUT_RSSI = 7
} RCInput_Capture_Channel;

/**
 * @brief   Input capture type selector.
 */
typedef enum {
    /**
     * @brief   CPPM input: all channels on one input line.
     */
    MODE_CPPM_INPUT = 1,
    /**
     * @brief   PWM input: all channels have its own input line.
     */
    MODE_PWM_INPUT = 2
} RCInput_Mode_Selector;

/**
 * @brief   Input capture channel role selector.
 */
typedef enum PACKED_VAR {
    /**
     * @brief   Unused role selector.
     */
    ROLE_OFF = 0,
    /**
     * @brief   Throttle role selector.
     */
    ROLE_THROTTLE = 1,
    /**
     * @brief   Pitch role selector.
     */
    ROLE_PITCH = 2,
    /**
     * @brief   Roll role selector.
     */
    ROLE_ROLL = 3,
    /**
     * @brief   Yaw role selector.
     */
    ROLE_YAW = 4,
    /**
     * @brief   Flight mode role selector.
     */
    ROLE_FLIGHT_MODE = 6,
    /**
     * @brief   Aux 1 role selector.
     */
    ROLE_AUX1 = 7,
    /**
     * @brief   Aux 2 role selector.
     */
    ROLE_AUX2 = 8,
    /**
     * @brief   Aux 3 role selector.
     */
    ROLE_AUX3 = 9
} Input_Role_Selector;

/**
 * @brief   Input capture channel type.
 */
typedef enum PACKED_VAR {
    /**
     * @brief   Analog type input.
     */
    TYPE_ANALOG = 1,
    /**
     * @brief   Tri-state type input.
     */
    TYPE_3_STATE = 2,
    /**
     * @brief   On/off type input.
     */
    TYPE_ON_OFF = 3
} Input_Type_Selector;

/**
 * @brief   Input capture data holder.
 */
typedef struct PACKED_VAR {
    /**
     * @brief   Active connection indicator.
     */
    uint32_t active_connection;
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
} RCInput_Data;

/**
 * @brief   Input capture settings holder.
 */
typedef struct {
    /**
     * @brief   Input capture mode selector.
     */
    uint32_t mode;
    /**
     * @brief   Input capture role selector.
     */
    Input_Role_Selector role[RCINPUT_MAX_NUMBER_OF_INPUTS];
    /**
     * @brief   Input capture type selector.
     */
    Input_Type_Selector type[RCINPUT_MAX_NUMBER_OF_INPUTS];
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
} RCInput_Settings;

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
float RCInputGetInputLevel(Input_Role_Selector role);
bool bActiveRCInputConnection(void);
RCInput_Data *ptrGetRCInputData(void);
RCInput_Settings *ptrGetRCInputSettings(void);
event_source_t *ptrGetRCInputEventSource(void);

#endif
