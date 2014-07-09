#ifndef __ESTIMATION_H
#define __ESTIMATION_H

#include "attitude_ekf.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define ESTIMATION_RESET_EVENTMASK                      EVENT_MASK(31)
#define ESTIMATION_NEW_ESTIMATION_EVENTMASK             EVENT_MASK(0)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void EstimationInit(void);
void ResetEstimation(void);
Attitude_Estimation_States *ptrGetAttitudeEstimationStates(void);
event_source_t *ptrGetEstimationEventSource(void);
quaternion_t MadgwickAHRSupdateIMU(vector3f_t g,
								   vector3f_t a,
								   quaternion_t q,
								   float beta,
								   float dt);
#endif
