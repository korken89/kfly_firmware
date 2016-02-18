#ifndef __VICON_H
#define __VICON_H

#include "geometry.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define VICON_DATA_EVENTMASK                EVENT_MASK(0)
#define VICON_MEASUREMENT_SIZE              (sizeof(vicon_measurement_t))

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
} vicon_measurement_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void ViconSupportInit(void);
event_source_t *ptrGetViconDataEventSource(void);
vicon_measurement_t *ptrGetViconMeasurement(void);
void GetCopyViconMeasurement(vicon_measurement_t *dest);
void vParseViconDataPackage(const uint8_t *payload, const uint8_t size);

#endif
