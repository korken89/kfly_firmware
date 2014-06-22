/* *
 *
 *
 * */

#include "linear_algebra.h"
#include "quaternion.h"
#include "trigonometry.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/*
 * @brief               Convert Pitch, Roll and Yaw to quaternion
 * 
 * @param[in] roll      Roll [rad]
 * @param[in] pitch     Pitch [rad]
 * @param[in] yaw       Yaw [rad]
 * @param[out] q        Pointer to save quaternion
 */
void euler2quat(float roll, float pitch, float yaw, quaternion_t *q)
{
    /* Prepare the angles for conversion to quaternions */
    pitch *= 0.5f;
    roll  *= 0.5f;
    yaw   *= 0.5f;

    q->q0 = fast_cos(roll) * fast_cos(pitch) * fast_cos(yaw) +
            fast_sin(roll) * fast_sin(pitch) * fast_sin(yaw);
    q->q1 = fast_sin(roll) * fast_cos(pitch) * fast_cos(yaw) - 
            fast_cos(roll) * fast_sin(pitch) * fast_sin(yaw);
    q->q2 = fast_cos(roll) * fast_sin(pitch) * fast_cos(yaw) + 
            fast_sin(roll) * fast_cos(pitch) * fast_sin(yaw);
    q->q3 = fast_cos(roll) * fast_cos(pitch) * fast_sin(yaw) - 
            fast_sin(roll) * fast_sin(pitch) * fast_cos(yaw);
}

/*
 * @brief               Converts a quaternion to a Direction Cosine Matrix.
 *
 * @param[out] R        Where to save the matrix.
 * @param[in] q         pointer to input quaternion.
 */
void q2dcm(float R[3][3], quaternion_t *q)
{
    /*
      R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2,       2*(q(2)*q(3)-q(1)*q(4)),       2*(q(2)*q(4)+q(1)*q(3));
               2*(q(2)*q(3)+q(1)*q(4)),   q(1)^2-q(2)^2+q(3)^2-q(4)^2,       2*(q(3)*q(4)-q(1)*q(2));
               2*(q(2)*q(4)-q(1)*q(3)),       2*(q(3)*q(4)+q(1)*q(2)),   q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    */
    float q0, q1, q2, q3;

    q0 = q->q0;
    q1 = q->q1;
    q2 = q->q2;
    q3 = q->q3;

    R[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    R[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    R[0][2] = 2.0f * (q1 * q3 + q0 * q2);

    R[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    R[1][1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    R[1][2] = 2.0f * (q2 * q3 - q0 * q1);

    R[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    R[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    R[2][2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

