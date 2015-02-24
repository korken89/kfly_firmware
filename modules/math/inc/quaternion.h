#ifndef __QUATERNION_H
#define __QUATERNION_H

#include <math.h>
#include "vector3.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PI ( 3.14159265359f )

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Quaternion definition.
 */
typedef struct
{
    /**
     * @brief   Scalar component.
     */
    float q0;
    /**
     * @brief   i component.
     */
    float q1;
    /**
     * @brief   j component.
     */
    float q2;
    /**
     * @brief   k component.
     */
    float q3;
} quaternion_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*
 * @brief               Converts a Generalized Rodrigues Parameter to a
 *                      quaternion.
 *
 * @param[in] v         GRP vector. 
 * @param[in] a         First scaling constant.
 * @param[in] f         Second scaling constant.
 * @return              Converted quaternion.
 */
static inline quaternion_t grp2q(const vector3f_t p,
                                 const float a,
                                 const float f)
{
    float sq, inv;
    quaternion_t q;

    sq = vector_norm(p);

    q.q0 = (-a * sq + f * sqrtf(f * f + (1.0f - a * a) * sq)) / (f * f + sq);

    inv = (a + q.q0) / f;
    q.q1 = p.x * inv;
    q.q2 = p.y * inv;
    q.q3 = p.z * inv;

    return q;
}

/*
 * @brief               Performs quaternion multiplication.
 *
 * @param[in] a         First quaternion to be multiplied.
 * @param[in] b         Second quaternion to be multiplied.
 * @return              Multiplied quaternion.
 */
static inline quaternion_t qmult(const quaternion_t a, const quaternion_t b)
{
    /**
     * q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
     *      a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
     *      a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
     *      a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)]
     * */
    quaternion_t r;

    r.q0 = a.q0 * b.q0 - a.q1 * b.q1 - a.q2 * b.q2 - a.q3 * b.q3;
    r.q1 = a.q0 * b.q1 + a.q1 * b.q0 + a.q2 * b.q3 - a.q3 * b.q2;
    r.q2 = a.q0 * b.q2 - a.q1 * b.q3 + a.q2 * b.q0 + a.q3 * b.q1;
    r.q3 = a.q0 * b.q3 + a.q1 * b.q2 - a.q2 * b.q1 + a.q3 * b.q0;

    return r;
}

/*
 * @brief               Conjugates a quaternion.
 *
 * @param[in] q         Quaternion to be conjugated.
 * @return              Conjugated quaternion.
 */
static inline quaternion_t qconj(const quaternion_t q)
{
    quaternion_t r;

    r.q0 =   q.q0;
    r.q1 = - q.q1;
    r.q2 = - q.q2;
    r.q3 = - q.q3;

    return r;
}

/*
 * @brief               Negates a quaternion.
 *
 * @param[in] q         Quaternion to be negated.
 * @return              Negated quaternion.
 */
static inline quaternion_t qneg(const quaternion_t q)
{
    quaternion_t r;

    r.q0 = - q.q0;
    r.q1 = - q.q1;
    r.q2 = - q.q2;
    r.q3 = - q.q3;

    return r;
}

/*
 * @brief               Calculates the norm of a quaternion.
 *
 * @param[in] q         Input quaternion.
 * @return              Quaternion norm.
 */
static inline float qnorm(const quaternion_t q)
{
    return sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
}

/*
 * @brief               Performs quaternion normalization.
 *
 * @param[in] q         Quaternion to be normalized.
 * @return              Normalized quaternion.
 */
static inline quaternion_t qnormalize(const quaternion_t q)
{
    quaternion_t r;
    float invNorm = 1.0f / sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);

    r.q0 = q.q0 * invNorm;
    r.q1 = q.q1 * invNorm;
    r.q2 = q.q2 * invNorm;
    r.q3 = q.q3 * invNorm;

    return r;
}

/*
 * @brief               Performs quaternion integration approximation. This
 *                      implementation assumes that omega * dt is small.
 *
 * @param[in] q_curr    Current quaternion to be integrated.
 * @param[in] omega     Angular rate.
 * @param[in] dt        Sampling time.
 * @return              Integrated quaternion.
 */
static inline quaternion_t qint(const quaternion_t q_curr,
                                const vector3f_t omega,
                                const float dt)
{
    quaternion_t q_step;
    float dx, dy, dz;

    dx = 0.5f * omega.x * dt;
    dy = 0.5f * omega.y * dt;
    dz = 0.5f * omega.z * dt;

    q_step.q0 = sqrtf(1.0f - dx * dx - dy * dy - dz * dz);
    q_step.q1 = dx;
    q_step.q2 = dy;
    q_step.q3 = dz;

    return qmult(q_step, q_curr);
}

/*
 * @brief               Converts an 4-element float array to a quaternion.
 *
 * @param[in] a         Array to be converted.
 * @return              The converted quaternion.
 */
static inline quaternion_t array2q(float a[4])
{
    quaternion_t q;

    q.q0 = a[0];
    q.q1 = a[1];
    q.q2 = a[2];
    q.q3 = a[3];

    return q;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void euler2quat(float roll, float pitch, float yaw, quaternion_t *q);
void q2dcm(float R[3][3], quaternion_t *q);

#endif
