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
 * @brief               Converts a quaternion to a Direction Cosine Matrix.
 *
 * @param[out] R        Pointer to the first element in the R matrix.
 * @param[in] q         Input quaternion.
 */
static inline void q2dcm(float R[3][3], const quaternion_t q)
{
    /*
      R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2,       2*(q(2)*q(3)-q(1)*q(4)),       2*(q(2)*q(4)+q(1)*q(3));
               2*(q(2)*q(3)+q(1)*q(4)),   q(1)^2-q(2)^2+q(3)^2-q(4)^2,       2*(q(3)*q(4)-q(1)*q(2));
               2*(q(2)*q(4)-q(1)*q(3)),       2*(q(3)*q(4)+q(1)*q(2)),   q(1)^2-q(2)^2-q(3)^2+q(4)^2];
    */
    const float q0sq = q.w * q.w;
    const float q1sq = q.x * q.x;
    const float q2sq = q.y * q.y;
    const float q3sq = q.z * q.z;

    R[0][0] = q0sq + q1sq - q2sq - q3sq;
    R[0][1] = 2.0f * (q.x * q.y - q.w * q.z);
    R[0][2] = 2.0f * (q.x * q.z + q.w * q.y);

    R[1][0] = 2.0f * (q.x * q.y + q.w * q.z);
    R[1][1] = q0sq - q1sq + q2sq - q3sq;
    R[1][2] = 2.0f * (q.y * q.z - q.w * q.x);

    R[2][0] = 2.0f * (q.x * q.z - q.w * q.y);
    R[2][1] = 2.0f * (q.y * q.z + q.w * q.x);
    R[2][2] = q0sq - q1sq - q2sq + q3sq;
}

/**
 * @brief           Rotates a vector v by the quaternion q.
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
    const float q0sq = q.w * q.w;
    const float q1sq = q.x * q.x;
    const float q2sq = q.y * q.y;
    const float q3sq = q.z * q.z;

    ret.x =       (q0sq + q1sq - q2sq - q3sq) * v.x;
    ret.x += (2.0f * (q.x * q.y - q.w * q.z)) * v.y;
    ret.x += (2.0f * (q.x * q.z + q.w * q.y)) * v.z;

    ret.y =  (2.0f * (q.x * q.y + q.w * q.z)) * v.x;
    ret.y +=      (q0sq - q1sq + q2sq - q3sq) * v.y;
    ret.y += (2.0f * (q.y * q.z - q.w * q.x)) * v.z;

    ret.z =  (2.0f * (q.x * q.z - q.w * q.y)) * v.x;
    ret.z += (2.0f * (q.y * q.z + q.w * q.x)) * v.y;
    ret.z +=      (q0sq - q1sq - q2sq + q3sq) * v.z;

    return ret;
}

/**
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

    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

    return r;
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
