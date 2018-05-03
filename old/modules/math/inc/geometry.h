/**
 *
 * Definitions for geometric quantities, based on the same naming as in ROS
 * geometry_msgs: http://wiki.ros.org/geometry_msgs
 *
 */

#ifndef __GEOMETRY_H
#define __GEOMETRY_H

#include "quaternion.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Definition of pose as position and orientation.
 */
typedef struct
{
    /**
     * @brief   Position as a vector3f in [m].
     */
    vector3f_t position;
    /**
     * @brief   Orientation as a unit quaternion.
     */
    quaternion_t orientation;
} pose_t;

/**
 * @brief   Alias of pose, transform is used for geometric operations.
 */
typedef pose_t transform_t;

/**
 * @brief   Definition of twist as linear and angular velocity.
 */
typedef struct
{
    /**
     * @brief   Linear veclocity as a vector3f in [m/s].
     */
    vector3f_t linear_velocity;
    /**
     * @brief   Angular veclocity as a vector3f in [rad/s].
     */
    vector3f_t angular_velocity;
} twist_t;

/**
 * @brief   Definition of accel as linear and angular acceleration.
 */
typedef struct
{
    /**
     * @brief   Linear acceleration as a vector3f in [m/s^2].
     */
    vector3f_t linear_acceleration;
    /**
     * @brief   Angular veclocity as a vector3f in [rad/s^2].
     */
    vector3f_t angular_acceleration;
} accel_t;

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief   Initializes a transformation with a position and an orientation.
 * @note    Does NOT rotate the position with the orientation, assumes a
 *          translation and rotation from a base link.
 *
 * @param[in] position      Initial position.
 * @param[in] orientation   Initial orientation.
 */
static inline transform_t TransformInit(const vector3f_t position,
                                        const quaternion_t orientation)
{
    transform_t ret;

    /* Save the position. */
    ret.position = position;

    /* Save the quaternion and make sure it is normalized. */
    ret.orientation = qnormalize(orientation);

    return ret;
}

/**
 * @brief   Chains two transforms to get the compound transformation.
 *
 * @param[in] base  Base link.
 * @param[in] next  Next link.
 *
 * @return  Returns the compound transform.
 */
static inline transform_t TransformChain(const transform_t base,
                                         const transform_t next)
{
    (void) base;
    (void) next;
    transform_t r;

    return r;
}

/*
 * @brief   Calculates the inverse transform.
 *
 * @param[in] t     Transform to be inverted.
 *
 * @return  The inverse transform.
 */
static inline transform_t TransformInverse(const transform_t t)
{
    transform_t r;

    r.orientation = qconj(t.orientation);
    r.position = qrotvector(r.orientation, vector_neg(t.position));

    return r;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif
