#ifndef __ESTIMATION_H
#define __ESTIMATION_H

#include "attitude_ekf.h"

/* Defines */
#define ESTIMATION_RESET_EVENT				EVENT_MASK(31) /* last event bit is 
															  set as reset bit
															*/

/* Typedefs */

/* Global variable defines */

/* Global function defines */
void EstimationInit(void);
void vTaskRunEstimation(void *);

void ResetEstimation(void);
Attitude_Estimation_States *ptrGetAttitudeEstimationStates(void);

#endif
