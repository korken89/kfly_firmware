#ifndef __VICON_H
#define __VICON_H

#include "quaternion.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define VICON_DATA_EVENTMASK                        EVENT_MASK(0)

#define VICON_QUATERNION_MASK                       (1<<7)
#define VICON_POSITION_MASK                         (1<<6)
#define VICON_VELOCITY_MASK                         (1<<5)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
typedef struct
{
    uint8_t available_data;
    uint32_t frame_number;
    quaternion_t attitude;
    vector3f_t position;
    vector3f_t velocity;
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
void vParseViconDataPackage(uint8_t *payload, uint8_t size);

#endif
