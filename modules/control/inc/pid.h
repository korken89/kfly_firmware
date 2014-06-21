#ifndef __PID_H
#define __PID_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PI_DATA_SIZE        (4*4)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
typedef struct
{
    float P_gain;
    float I_gain;
    float I_limit;
    float I_state;
} PI_Data;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vInitPIController(PI_Data *pi_settings,
                       float P_gain,
                       float I_gain,
                       float I_limit);
void vUpdatePISettings(PI_Data *pi_settings,
                       float P_gain,
                       float I_gain,
                       float I_limit);
float fPIUpdate(PI_Data *pi, float error, float dt);

#endif
