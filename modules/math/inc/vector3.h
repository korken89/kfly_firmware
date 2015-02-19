#ifndef __VECTOR_H
#define __VECTOR_H

#include <math.h>

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   3 dimensional vector definition.
 */
typedef struct
{
    /**
     * @brief   x component.
     */
    float x;
    /**
     * @brief   y component.
     */
    float y;
    /**
     * @brief   z component.
     */
    float z;
} vector3f_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*
 * @brief               Converts an 3-element float array to a vector.
 *
 * @param[in] a         Array to be converted.
 * @return              The converted vector.
 */
static inline vector3f_t array_to_vector(float a[3])
{
    vector3f_t r;

    r.x = a[0];
    r.y = a[1];
    r.z = a[2];

    return r;
}

/*
 * @brief               Adds two vectors.
 *
 * @param[in] v         First vector to be added. 
 * @param[in] w         Second vector to be added. 
 * @return              The sum of the two vectors.
 */
static inline vector3f_t vector_add(const vector3f_t v, const vector3f_t w)
{
    vector3f_t r;

    r.x = v.x + w.x;
    r.y = v.y + w.y;
    r.z = v.z + w.z;

    return r;
}

/*
 * @brief               Subtract two vectors.
 *
 * @param[in] v         First vector to be subtracted. 
 * @param[in] w         Second vector to be subtraced. 
 * @return              The difference of the two vectors.
 */
static inline vector3f_t vector_sub(const vector3f_t v, const vector3f_t w)
{
    vector3f_t r;

    r.x = v.x - w.x;
    r.y = v.y - w.y;
    r.z = v.z - w.z;

    return r;
}

/*
 * @brief               Scale a vector.
 *
 * @param[in] v         Vector to be scaled. 
 * @param[in] scale     Scaling constant.
 * @return              The scaled vector.
 */
static inline vector3f_t vector_scale(const vector3f_t v, const float scale)
{
    vector3f_t r;

    r.x = v.x * scale;
    r.y = v.y * scale;
    r.z = v.z * scale;

    return r;
}

/*
 * @brief               Performs the dot product of two vectors.
 *
 * @param[in] v         First vector to be dotted. 
 * @param[in] w         Second vector to be dotted. 
 * @return              The dot product of the two vectors.
 */
static inline float vector_dot_product(const vector3f_t v, const vector3f_t w)
{
    float r;

    r = v.x * w.x + v.y * w.y + v.z * w.z;

    return r;
}

/*
 * @brief               Performs the cross product of two vectors.
 *
 * @param[in] v         First vector to be crossed. 
 * @param[in] w         Second vector to be crossed. 
 * @return              The cross product of the two vectors.
 */
static inline vector3f_t vector_cross_product(const vector3f_t v,
                                              const vector3f_t w)
{
    vector3f_t r;

    r.x = v.y * w.z - v.z * w.y;
    r.y = v.z * w.x - v.x * w.z;
    r.z = v.x * w.y - v.y * w.x;

    return r;
}

/*
 * @brief               Performs vector rotation.
 *
 * @param[in] R         Rotation matrix. 
 * @param[in] v         Vector to be rotated. 
 * @return              The rotated vector.
 */
static inline vector3f_t vector_rotation(float R[3][3], const vector3f_t v)
{
    vector3f_t rot;

    rot.x = R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z;
    rot.y = R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z;
    rot.z = R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z;

    return rot;
}

/*
 * @brief               Performs vector rotation (R is transposed).
 *
 * @param[in] R         Rotation matrix. 
 * @param[in] v         Vector to be rotated. 
 * @return              The rotated vector.
 */
static inline vector3f_t vector_rotation_transposed(float R[3][3],
                                                    const vector3f_t v)
{
    vector3f_t rot;

    rot.x = R[0][0] * v.x + R[1][0] * v.y + R[2][0] * v.z;
    rot.y = R[0][1] * v.x + R[1][1] * v.y + R[2][1] * v.z;
    rot.z = R[0][2] * v.x + R[1][2] * v.y + R[2][2] * v.z;

    return rot;
}

/*
 * @brief               Calculates the norm of a vector.
 *
 * @param[in] v         Input vector. 
 * @return              The vector norm.
 */
static inline float vector_norm(const vector3f_t v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/


#endif
