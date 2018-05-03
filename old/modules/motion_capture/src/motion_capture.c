/* *
 *
 *
 *
 * */

#include "ch.h"
#include "hal.h"
#include "motion_capture.h"
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
static EVENTSOURCE_DECL(new_mc_frame_es);
motion_capture_t mc_frame;

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes the new motion capture frame events.
 */
void MotionCaptureInit(void)
{
    /* Initialize motion capture frame event source */
    osalEventObjectInit(&new_mc_frame_es);

    /* Initialize the data structure */
    mc_frame.frame_number = 0;

    mc_frame.pose.orientation.w = 1.0f;
    mc_frame.pose.orientation.x = 0.0f;
    mc_frame.pose.orientation.y = 0.0f;
    mc_frame.pose.orientation.z = 0.0f;

    mc_frame.pose.position.x = 0.0f;
    mc_frame.pose.position.y = 0.0f;
    mc_frame.pose.position.z = 0.0f;
}

/**
 * @brief       Returns the pointer to the new Motion Capture frame event source.
 *
 * @return      Pointer to the motion capture frame event source.
 */
event_source_t *ptrGetMotionCaptureDataEventSource(void)
{
    return &new_mc_frame_es;
}

/**
 * @brief       Returns the pointer to the motion capture frame holder.
 *
 * @return      Pointer to the motion capture frame holder.
 */
motion_capture_t *ptrGetMotionCaptureFrame(void)
{
    return &mc_frame;
}

/**
 * @brief               Copies the motion capture frame holder to a chosen
 *                      destination.
 *
 * @param[out] dest     Pointer to the destination location.
 */
void GetCopyMotionCaptureFrame(motion_capture_t *dest)
{
    /* Lock while copying the data. */
    osalSysLock();

    memcpy((uint8_t *)dest,
           (uint8_t *)&mc_frame,
           sizeof(motion_capture_t));

    osalSysUnlock();
}

/**
 * @brief               Parses a payload from the serial communication for
 *                      motion capture data.
 *
 * @param[in] payload   Pointer to the payload location.
 * @param[in] size      Size of the payload.
 */
void vParseMotionCaptureDataPackage(const uint8_t *payload, const uint8_t size)
{
    /* If sizes are matching start decoding. */
    if (size == MOTION_CAPTURE_MEASUREMENT_SIZE)
    {
        /* Lock while saving the data. */
        osalSysLock();

        /* Save data. */
        memcpy((uint8_t *)&mc_frame,
               payload,
               MOTION_CAPTURE_MEASUREMENT_SIZE);

        chEvtBroadcastFlagsI(&new_mc_frame_es, MOTION_CAPTURE_DATA_EVENTMASK);

        /* osalOsRescheduleS() must be called after a chEvtBroadcastFlagsI() */
        osalOsRescheduleS();

        osalSysUnlock();
    }
}
