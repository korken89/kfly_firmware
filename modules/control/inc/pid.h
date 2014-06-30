#ifndef __PID_H
#define __PID_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PI_DATA_SIZE        (sizeof(PI_Data))

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   PI controller data structure.
 */
typedef struct PACKED_VAR
{
	/**
 	 * @brief   Controller proportional gain.
 	 */
    float P_gain;
    /**
 	 * @brief   Controller integral gain.
 	 */
    float I_gain;
    /**
 	 * @brief   Controller integral limit.
 	 */
    float I_limit;
    /**
 	 * @brief   Current controller integral state.
 	 */
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
