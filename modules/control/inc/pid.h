#ifndef __PID_H
#define __PID_H

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PI_DATA_SIZE        (sizeof(pi_data_t))
#define PI_SETTINGS_SIZE    (3*sizeof(float))

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
} pi_data_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void vInitPIController(pi_data_t *pi_settings,
                       float P_gain,
                       float I_gain,
                       float I_limit);
void vUpdatePISettings(pi_data_t *pi_settings,
                       float P_gain,
                       float I_gain,
                       float I_limit);
float fPIUpdate(pi_data_t *pi, float error, float dt);
float fPIUpdate_BackCalculationSaturation(pi_data_t *pi,
                                          float error,
                                          float u_max,
                                          float u_min,
                                          float dt);

#endif
