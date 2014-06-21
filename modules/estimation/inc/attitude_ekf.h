#ifndef __ATTITUDE_EKF_H
#define __ATTITUDE_EKF_H

#include <math.h>
#include "quaternion.h"
#include "linear_algebra.h"
#include "trigonometry.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define ESTIMATION_DT   (0.005f)

/* Process covariances */
#define SQ_Q            (0.01f)                     /* Quaternion error
                                                       covariance */
#define SQ_B            (0.001f * ESTIMATION_DT)    /* Gyro bias covariance */ 

/* Observation covariances */
#define SR_A            (100.0f)                    /* Acceleration error
                                                       covariance */
#define SR_T            (1000.0f)                   /* Magnetometer error 
                                                       covariance */

/* Starting error covariance */
#define S_P             (10.0f)                     /* Starting value of the
                                                       diagonal of P */

/* Calculate the square-root factors 
 * Generated from Matlab, do not change */
#define SQ_1            (sqrtf(SQ_Q))
#define SQ_2            (0.5f * sqrtf(SQ_B * (4.0f * SQ_Q - SQ_B) / SQ_Q))
#define SQ_3            (-SQ_B / (2.0f * sqrtf(SQ_Q)))
#define SR_1            (sqrtf(SR_A))
#define SR_2            (sqrtf(SR_T))
#define SP_1            (sqrtf(S_P))
/* End of Matlab generated code */

/* GRP settings */
#define GRP_A           (1.0f)
#define GRP_F           (2.0f * (GRP_A + 1.0f))     /* This choice of GRP_F
                                                       makes the small angle 
                                                       approximation equal 
                                                       the angle  */

/* Sizes */
#define ESTIMATION_ATTITUDE_STATES_SIZE     (10*4)

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Attitude estimation states.
 */
typedef struct
{
    /**
     * @brief   Attitude quaternion.
     */
    quaternion_t q;
    /**
     * @brief   Angular rate in rad/s.
     */
    vector3f_t w;
    /**
     * @brief   Angular rate bias in rad/s.
     */
    vector3f_t wb;
} Attitude_Estimation_States;

/**
 * @brief   EKF matrices.
 */
typedef struct
{
    /**
     * @brief   Square-root error covariance matrix.
     */
    float Sp[6][6];
     /**
     * @brief   Temporary matrix 1.
     */
    float T1[6][6];
     /**
     * @brief   Square-root innovation covariance matrix.
     */
    float Ss[3][3];
     /**
     * @brief   Temporary matrix 2.
     */
    float T2[3][3];
     /**
     * @brief   Temporary matrix 3.
     */
    float T3[3][6];
} Attitude_Estimation_Data;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void AttitudeEstimationInit(Attitude_Estimation_States *states,
                            Attitude_Estimation_Data *settings,
                            quaternion_t *start_attitude,
                            vector3f_t *start_bias);
void GenerateStartingGuess(vector3f_t *acc,
                           vector3f_t *mag, 
                           quaternion_t *attitude_guess);
void InnovateAttitudeEKF(Attitude_Estimation_States *states,
                         Attitude_Estimation_Data *settings, 
                         float gyro[3],
                         float acc[3],
                         float mag[3],
                         float beta,
                         float u_sum,
                         float dt);

#endif
