#ifndef __CONTROL_H
#define __CONTROL_H

#include "control_definitions.h"
#include "control_reference.h"
#include "arming.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

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

void ControlInit(void);
void vUpdateControlAction(quaternion_t *q_m, vector3f_t *omega_m, float dt);
control_reference_t *ptrGetControlReferences(void);
control_data_t *ptrGetControlData(void);
control_limits_t *ptrGetControlLimits(void);
output_mixer_t *ptrGetOutputMixer(void);
void GetControlParameters(control_parameters_t *param);
void SetControlParameters(control_parameters_t *param);

#endif
