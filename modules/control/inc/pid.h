#ifndef __PID_H
#define __PID_H

/* Defines */
#define PI_DATA_SIZE        (4*4)

/* Typedefs */
typedef struct
{
    float P_gain;
    float I_gain;
    float I_limit;
    float I_state;
} PI_Data_Type;

/* Global variable defines */

/* Global function defines */
void vInitPIController(PI_Data_Type *, float, float, float);
void vUpdatePISettings(PI_Data_Type *, float, float, float);
float fPIUpdate(PI_Data_Type *, float, float);

#endif
