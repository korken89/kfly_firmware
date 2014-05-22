#ifndef __ESTIMATION_H
#define __ESTIMATION_H



/* Defines */
#define ATTITUDE_ESTIMATION_STATES_SIZE     28

/* Typedefs */

/* Global variable defines */

/* Global function defines */
void EstimationInit(void);
void vTaskRunEstimation(void *);

void ResetEstimation(Attitude_Estimation_States *states,
                     Attitude_Estimation_Data *data);
Attitude_Estimation_States *ptrGetAttitudeEstimationStates(void);

#endif
