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

/* TODO: Add transform operations. */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif
