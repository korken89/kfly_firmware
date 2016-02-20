#ifndef __QUATERNION_H
#define __QUATERNION_H

#include <math.h>
#include "vector3.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define UNIT_QUATERNION ((const quaternion_t){.w = 1.0f, .x = 0.0f,         \\
                                              .y = 0.0f, .z = 0,0f})

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
    float w;
    /**
     * @brief   i component.
     */
    float x;
    /**
     * @brief   j component.
     */
    float y;
    /**
     * @brief   k component.
     */
    float z;
} quaternion_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
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

    q.w = (-a * sq + f * sqrtf(f * f + (1.0f - a * a) * sq)) / (f * f + sq);

    inv = (a + q.w) / f;
    q.x = p.x * inv;
    q.y = p.y * inv;
    q.z = p.z * inv;

    return q;
}

/**
 * @brief               Conjugates a quaternion.
 *
 * @param[in] q         Quaternion to be conjugated.
 * @return              Conjugated quaternion.
 */
static inline quaternion_t qconj(const quaternion_t q)
{
    quaternion_t r;

    r.w =   q.w;
    r.x = - q.x;
    r.y = - q.y;
    r.z = - q.z;

    return r;
}

/**
 * @brief               Negates a quaternion.
 *
 * @param[in] q         Quaternion to be negated.
 * @return              Negated quaternion.
 */
static inline quaternion_t qneg(const quaternion_t q)
{
    quaternion_t r;

    r.w = - q.w;
    r.x = - q.x;
    r.y = - q.y;
    r.z = - q.z;

    return r;
}

/**
 * @brief               Calculates the norm of a quaternion.
 *
 * @param[in] q         Input quaternion.
 * @return              Quaternion norm.
 */
static inline float qnorm(const quaternion_t q)
{
    return sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

/**
 * @brief               Performs quaternion normalization.
 *
 * @param[in] q         Quaternion to be normalized.
 * @return              Normalized quaternion.
 */
static inline quaternion_t qnormalize(const quaternion_t q)
{
    quaternion_t r;
    float invNorm = 1.0f / sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

    r.w = q.w * invNorm;
    r.x = q.x * invNorm;
    r.y = q.y * invNorm;
    r.z = q.z * invNorm;

    return r;
}

/**
 * @brief               Performs quaternion multiplication p * q.
 *
 * @param[in] p         First quaternion to be multiplied.
 * @param[in] q         Second quaternion to be multiplied.
 * @return              Multiplied quaternion.
 */
static inline quaternion_t qmult(const quaternion_t p, const quaternion_t q)
{
    /*
     * Quaternion multiplication from "A survey of attitude representations" by
     * Malcom D. Shuster (p. 473). It is not written in the Hamiltonian order,
     * it is rather a convenction resulting in multiplications of quaternions
     * in "natural order". This is in accordance with the JPL Proposed Standard
     * Conventions [W. G. Breckenridge, “Quaternions - Proposed Standard
     * Conventions,” JPL, Tech. Rep. INTEROFFICE MEMORANDUM IOM 343-79-1199,
     * 1999].
     */

    quaternion_t r;

    r.w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
    r.x = - p.x * q.w + p.w * q.x + p.z * q.y + p.y * q.z;
    r.y = p.y * q.w - p.z * q.x + p.w * q.y + p.x * q.z;
    r.z = p.z * q.w + p.y * q.x - p.x * q.y + p.w * q.z;

    return r;
}

/**
 * @brief           Converts a quaternion to a rotation matrix, where q
 *                  represents the rotation from frame A to frame B. Then
 *                  V^B = R * V^A
 *
 * @param[out] R    Pointer to the first element in the R matrix.
 * @param[in] q     Input quaternion.
 */
static inline void q2dcm(float R[3][3], const quaternion_t q)
{
    /*
     * Quaternion rotation matrix from "A survey of attitude representations" by
     * Malcom D. Shuster (p. 473). It is not written in the Hamiltonian order,
     * it is rather a convenction resulting in multiplications of quaternions
     * in "natural order". This is in accordance with the JPL Proposed Standard
     * Conventions [W. G. Breckenridge, “Quaternions - Proposed Standard
     * Conventions,” JPL, Tech. Rep. INTEROFFICE MEMORANDUM IOM 343-79-1199,
     * 1999].
     */

    const float qwsq = q.w * q.w;
    const float qxsq = q.x * q.x;
    const float qysq = q.y * q.y;
    const float qzsq = q.z * q.z;

    R[0][0] = qwsq + qxsq - qysq - qzsq;
    R[0][1] = 2.0f * (q.x * q.y + q.w * q.z);
    R[0][2] = 2.0f * (q.x * q.z - q.w * q.y);

    R[1][0] = 2.0f * (q.x * q.y - q.w * q.z);
    R[1][1] = qwsq - qxsq + qysq - qzsq;
    R[1][2] = 2.0f * (q.y * q.z + q.w * q.x);

    R[2][0] = 2.0f * (q.x * q.z + q.w * q.y);
    R[2][1] = 2.0f * (q.y * q.z - q.w * q.x);
    R[2][2] = qwsq - qxsq - qysq + qzsq;
}

/**
 * @brief           Rotates a vector v by the quaternion q, that is if q
 *                  represents the rotation from frame A to frame B then
 *                  v^B = q * v^A * q^-1.
 *
 * @param[in] q     Quaternion rotation.
 * @param[in] v     Input vector.
 *
 * @return          The rotated vector.
 */
static inline vector3f_t qrotvector(const quaternion_t q, const vector3f_t v)
{
    vector3f_t ret;

    /* Rotation from the rotation matrix equations directly applied on v. */
    const float qwsq = q.w * q.w;
    const float qxsq = q.x * q.x;
    const float qysq = q.y * q.y;
    const float qzsq = q.z * q.z;

    ret.x =       (qwsq + qxsq - qysq - qzsq) * v.x;
    ret.x += (2.0f * (q.x * q.y + q.w * q.z)) * v.y;
    ret.x += (2.0f * (q.x * q.z - q.w * q.y)) * v.z;

    ret.y =  (2.0f * (q.x * q.y - q.w * q.z)) * v.x;
    ret.y +=      (qwsq - qxsq + qysq - qzsq) * v.y;
    ret.y += (2.0f * (q.y * q.z + q.w * q.x)) * v.z;

    ret.z =  (2.0f * (q.x * q.z + q.w * q.y)) * v.x;
    ret.z += (2.0f * (q.y * q.z - q.w * q.x)) * v.y;
    ret.z +=      (qwsq - qxsq - qysq + qzsq) * v.z;

    return ret;
}

/**
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

    /* TODO: Rewrite to the zeroth order integrator. */
    dx = 0.5f * omega.x * dt;
    dy = 0.5f * omega.y * dt;
    dz = 0.5f * omega.z * dt;

    /* Small angle approximation. */
    q_step.w = sqrtf(1.0f - dx * dx - dy * dy - dz * dz);
    q_step.x = dx;
    q_step.y = dy;
    q_step.z = dz;

    return qmult(q_step, q_curr);
}

/**
 * @brief               Converts an 4-element float array to a quaternion.
 *
 * @param[in] a         Array to be converted.
 * @return              The converted quaternion.
 */
static inline quaternion_t array2q(float a[4])
{
    quaternion_t q;

    q.w = a[0];
    q.x = a[1];
    q.y = a[2];
    q.z = a[3];

    return q;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void euler2quat(const float roll,
                const float pitch,
                const float yaw,
                quaternion_t *q);

#endif
