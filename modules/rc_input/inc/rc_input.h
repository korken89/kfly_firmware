#ifndef __RC_INPUT_H
#define __RC_INPUT_H

/* Defines */
#define CAPTURE_TIMER_RATE          1000000
#define MAX_NUMBER_OF_INPUTS        12
#define CPPM_SYNC_LIMIT_MIN         3000    /* 3 ms */
#define CPPM_SYNC_LIMIT_MAX         30000   /* 30 ms */
#define RSSI_THRESHOLD_PERCENT      5       /* RSSI threshold in percent */
#define RSSI_TIMEOUT                100     /* Number of bad RSSI measurements 
                                               to disable connection */
#define RCINPUT_NO_CON_TIMEOUT_MS   500
#define RCINPUT_LOST_EVENTMASK      EVENT_MASK(0)
#define RCINPUT_ACTIVE_EVENTMASK    EVENT_MASK(1)
#define RCINPUT_NEWINPUT_EVENTMASK  EVENT_MASK(2)
#define RCINPUT_ROLE_TO_INDEX_BITS  4
#define RCINPUT_ROLE_TO_INDEX_MASK  0x0f

#define RCINPUT_DATA_SIZE           (10 + 2 * MAX_NUMBER_OF_INPUTS)
#define RCINPUT_SETTINGS_SIZE       (4 + 8 * MAX_NUMBER_OF_INPUTS)

/* Global variable defines */

/* Typedefs */
typedef enum {
    INPUT_CH1 = 0,
    INPUT_CH2 = 1,
    INPUT_CH3 = 2,
    INPUT_CH4 = 3,
    INPUT_CH5 = 4,
    INPUT_CH6 = 5,
    INPUT_CPPM = 6,
    INPUT_RSSI = 7
} RCInput_Capture_Channel;

typedef enum {
    MODE_CPPM_INPUT = 1,
    MODE_PWM_INPUT = 2
} RCInput_Mode_Selector;

typedef enum PACKED_VAR {
    ROLE_OFF = 0,
    ROLE_THROTTLE = 1,
    ROLE_PITCH = 2,
    ROLE_ROLL = 3,
    ROLE_YAW = 4,
    ROLE_FLIGHT_MODE = 6,
    ROLE_AUX1 = 7,
    ROLE_AUX2 = 8,
    ROLE_AUX3 = 9
} Input_Role_Selector;

typedef enum PACKED_VAR {
    TYPE_ANALOG = 1,
    TYPE_3_STATE = 2,
    TYPE_ON_OFF = 3
} Input_Type_Selector;

typedef struct {
    uint32_t active_connection;
    uint16_t number_active_connections;
    uint16_t value[MAX_NUMBER_OF_INPUTS];
    uint16_t rssi;
    uint16_t rssi_frequency;
} RCInput_Data;

typedef struct {
    uint32_t mode;
    Input_Role_Selector role[MAX_NUMBER_OF_INPUTS];
    Input_Type_Selector type[MAX_NUMBER_OF_INPUTS];
    uint16_t ch_center[MAX_NUMBER_OF_INPUTS];
    uint16_t ch_top[MAX_NUMBER_OF_INPUTS];
    uint16_t ch_bottom[MAX_NUMBER_OF_INPUTS];
} RCInput_Settings;

/* Global function defines */
msg_t RCInputInit(RCInput_Mode_Selector mode);
float RCInputGetInputLevel(Input_Role_Selector role);
RCInput_Data *ptrGetRCRawInput(void);
RCInput_Settings *ptrGetRCInputSettings(void);
event_source_t *ptrGetRCInputEventSource(void);

#endif
