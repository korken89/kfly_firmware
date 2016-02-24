#ifndef __MOTION_CAPTURE_H
#define __MOTION_CAPTURE_H

#include "geometry.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define MOTION_CAPTURE_DATA_EVENTMASK       EVENT_MASK(0)
#define MOTION_CAPTURE_MEASUREMENT_SIZE     (sizeof(motion_capture_t))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Data structure for a measurement from a motion capture system.
 */
typedef struct PACKED_VAR
{
    /**
     * @brief   Frame number from the motion capture system.
     */
    uint32_t frame_number;
    /**
     * @brief   Pose from the motion capture system.
     */
    pose_t pose;
} motion_capture_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void MotionCaptureInit(void);
event_source_t *ptrGetMotionCaptureDataEventSource(void);
motion_capture_t *ptrGetMotionCaptureFrame(void);
void GetCopyMotionCaptureFrame(motion_capture_t *dest);
void vParseMotionCaptureDataPackage(const uint8_t *payload, const uint8_t size);

#endif
