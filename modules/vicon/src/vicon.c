/* *
 *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "vicon.h"
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
static EVENTSOURCE_DECL(new_vicon_data_es);
vicon_measurement_t vicon_measurement;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

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

    /* Initialize the data structure */
    vicon_measurement.frame_number = 0;

    vicon_measurement.pose.orientation.w = 1.0f;
    vicon_measurement.pose.orientation.x = 0.0f;
    vicon_measurement.pose.orientation.y = 0.0f;
    vicon_measurement.pose.orientation.z = 0.0f;

    vicon_measurement.pose.position.x = 0.0f;
    vicon_measurement.pose.position.y = 0.0f;
    vicon_measurement.pose.position.z = 0.0f;
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

    memcpy((uint8_t *)dest,
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
void vParseViconDataPackage(const uint8_t *payload, const uint8_t size)
{
    /* If sizes are matching start decoding. */
    if (size == VICON_MEASUREMENT_SIZE)
    {
        /* Lock while saving the data. */
        osalSysLock();
        /* The unlock is called in vBroadcastNewViconDataAvailable() */

        /* Save data. */
        memcpy((uint8_t *)&vicon_measurement,
               payload,
               VICON_MEASUREMENT_SIZE);

        chEvtBroadcastFlagsI(&new_vicon_data_es, VICON_DATA_EVENTMASK);

        /* osalOsRescheduleS() must be called after a chEvtBroadcastFlagsI() */
        osalOsRescheduleS();

        osalSysUnlock();
    }
}
