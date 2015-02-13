/* *
 *
 * 
 *
 * */

#include "ch.h"
#include "hal.h"
#include "vicon.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/
#define VICON_QUATERNION_SIZE               (16)
#define VICON_POSITION_SIZE                 (12)
#define VICON_VELOCITY_SIZE                 (12)

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/
static EVENTSOURCE_DECL(new_vicon_data_es);
static vicon_measurement_t vicon_measurement;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief                   A generic function to save data. Locks the RTOS
 *                          while saving.
 *
 * @param[in] save_location Pointer to the location where the data shall be
 *                          saved.
 * @param[in] buffer        Pointer to the buffer where to save from.
 * @param[in] data_length   Number of bytes to save
 */
static void GenericSaveData(uint8_t *save_location,
                            uint8_t *buffer,
                            uint32_t data_length)
{
    uint32_t i;

    /* Save the string */
    for (i = 0; i < data_length; i++)
            save_location[i] = buffer[i];
}

/**
 * @brief   Broadcast the New Vicon Data events.
 */
static void vBroadcastNewViconDataAvailable(void)
{
    /* The lock is called in ParseViconDataPackage() */

    if (chEvtIsListeningI(&new_vicon_data_es))
    {
        chEvtBroadcastFlagsI(&new_vicon_data_es, VICON_DATA_EVENTMASK);

        /* osalOsRescheduleS() must be called after a chEvtBroadcastFlagsI() */
        osalOsRescheduleS();
    }

    osalSysUnlock();
}

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the New Vicon Data events.
 */
void ViconSupportInit(void)
{
    /* Initialize New Vicon Data event source */
    osalEventObjectInit(&new_vicon_data_es);
}

/**
 * @brief       Returns the pointer to the New Vicon Data event source.
 *
 * @return      Pointer to the New Vicon Data event source.
 */
event_source_t *ptrGetViconDataEventSource(void)
{
    return &new_vicon_data_es;
}

/**
 * @brief       Returns the pointer to the Vicon Data holder.
 *
 * @return      Pointer to the Vicon Data holder.
 */
vicon_measurement_t *ptrGetViconMeasurement(void)
{
    return &vicon_measurement;
}

/**
 * @brief               Copies the Vicon Data holder to a chosen destination.
 *
 * @param[out] dest     Pointer to the destination location.
 */
void GetCopyViconMeasurement(vicon_measurement_t *dest)
{
    /* Lock while copying the data. */
    osalSysLock();

    GenericSaveData((uint8_t *)dest,
                    (uint8_t *)&vicon_measurement,
                    sizeof(vicon_measurement_t));

    osalSysUnlock();
}

/**
 * @brief               Parses a payload from the serial communication for
 *                      Vicon data.
 *
 * @param[in] payload   Pointer to the payload location.
 * @param[in] size      Size of the payload.
 */
void vParseViconDataPackage(uint8_t *payload, uint8_t size)
{
    uint8_t expected_size = 0, current_index = 1;
    uint8_t available_data = payload[0];

    /* Check the expected size for the data. */
    if (available_data & VICON_QUATERNION_MASK)
        expected_size += VICON_QUATERNION_SIZE;
        /* Quaternion data available: 4 floats */

    if (available_data & VICON_POSITION_MASK)
        expected_size += VICON_POSITION_SIZE;
        /* Position data available: 3 floats */

    if (available_data & VICON_VELOCITY_MASK)
        expected_size += VICON_VELOCITY_SIZE;
        /* Velocity data available: 3 floats */

    /* Sizes are matching start decoding. */
    if (size == expected_size)
    {
        /* Lock while saving the data. */
        osalSysLock();
        /* The unlock is called in vBroadcastNewViconDataAvailable() */

        /* Save quaternion. */
        if (available_data & VICON_QUATERNION_MASK)
        {
            GenericSaveData((uint8_t *)&vicon_measurement.attitude,
                            &payload[current_index],
                            VICON_QUATERNION_SIZE);

            /* Add the offset for the next data packet. */
            current_index += VICON_QUATERNION_SIZE;
        }

        /* Save position. */
        if (available_data & VICON_POSITION_MASK)
        {
            GenericSaveData((uint8_t *)&vicon_measurement.position,
                            &payload[current_index],
                            VICON_POSITION_SIZE);

            /* Add the offset for the next data packet. */
            current_index += VICON_POSITION_SIZE;
        }

        /* Save velocity. */
        if (available_data & VICON_VELOCITY_MASK)
        {
            GenericSaveData((uint8_t *)&vicon_measurement.velocity,
                            &payload[current_index],
                            VICON_VELOCITY_SIZE);
        }

        vicon_measurement.available_data = available_data;

        vBroadcastNewViconDataAvailable();
    }
}