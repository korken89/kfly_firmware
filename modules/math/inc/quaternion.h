#ifndef __QUATERNION_H
#define __QUATERNION_H

/* Standard includes */


/* System includes */

/* Scheduler includes */

/* KFly includes */

/* Includes */

/* Defines */
#define PI ( 3.14159265359f )

/* Typedefs */
typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
} quaternion_t;

typedef struct
{
	float x;
	float y;
	float z;
} vector3f_t;

/*
 * @brief				Adds two vectors.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			First vector to be added. 
 * @param[in] w			Second vector to be added. 
 * @return 				The sum of the two vectors.
 */
static inline vector3f_t vector_add(vector3f_t v, vector3f_t w)
{
	vector3f_t r;

	r.x = v.x + w.x;
	r.y = v.y + w.y;
	r.z = v.z + w.z;

 	return r;
}

/*
 * @brief				Subtract two vectors.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			First vector to be subtracted. 
 * @param[in] w			Second vector to be subtraced. 
 * @return 				The difference of the two vectors.
 */
static inline vector3f_t vector_sub(vector3f_t v, vector3f_t w)
{
	vector3f_t r;

	r.x = v.x - w.x;
	r.y = v.y - w.y;
	r.z = v.z - w.z;

 	return r;
}

/*
 * @brief				Scale a vector.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			Vector to be scaled. 
 * @param[in] scale		Scaling constant.
 * @return 				The scaled vector.
 */
static inline vector3f_t vector_scale(vector3f_t v, float scale)
{
	vector3f_t r;

	r.x = v.x * scale;
	r.y = v.y * scale;
	r.z = v.z * scale;

 	return r;
}

/*
 * @brief				Performs the dot product of two vectors.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			First vector to be dotted. 
 * @param[in] w			Second vector to be dotted. 
 * @return 				The dot product of the two vectors.
 */
static inline float vector_dot_product(vector3f_t v, vector3f_t w)
{
	float r;

	r = v.x * w.x + v.y * w.y + v.z * w.z;

 	return r;
}

/*
 * @brief				Performs the cross product of two vectors.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			First vector to be crossed. 
 * @param[in] w			Second vector to be crossed. 
 * @return 				The cross product of the two vectors.
 */
static inline vector3f_t vector_cross_product(vector3f_t v, vector3f_t w)
{
	vector3f_t r;

	r.x = v.y * w.z - v.z * w.y;
 	r.y = v.z * w.x - v.x * w.z;
 	r.z = v.x * w.y - v.y * w.x;

 	return r;
}

/*
 * @brief				Performs vector rotation.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] R			Rotation matrix. 
 * @param[in] v			Vector to be rotated. 
 * @return 				The rotated vector.
 */
static inline vector3f_t vector_rotation(float R[3][3], vector3f_t v)
{
	vector3f_t rot;

	rot.x = R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z;
 	rot.y = R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z;
 	rot.z = R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z;

 	return rot;
}

/*
 * @brief				Performs vector rotation (R is transposed).
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] R			Rotation matrix. 
 * @param[in] v			Vector to be rotated. 
 * @return 				The rotated vector.
 */
static inline vector3f_t vector_rotation_transposed(float R[3][3], vector3f_t v)
{
	vector3f_t rot;

	rot.x = R[0][0] * v.x + R[1][0] * v.y + R[2][0] * v.z;
 	rot.y = R[0][1] * v.x + R[1][1] * v.y + R[2][1] * v.z;
 	rot.z = R[0][2] * v.x + R[1][2] * v.y + R[2][2] * v.z;

 	return rot;
}

/*
 * @brief				Calculates the norm of a vector.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			Input vector. 
 * @return 				The vector norm.
 */
static inline float vector_norm(vector3f_t v)
{
 	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/*
 * @brief				Converts a Generalized Rodrigues Parameter to a
 * 						quaternion.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] v			GRP vector. 
 * @param[in] a 		First scaling constant.
 * @param[in] f 		Second scaling constant.
 * @return 				Converted quaternion.
 */
static inline quaternion_t grp2q(vector3f_t p, const float a, const float f)
{
	float sq, inv;
	quaternion_t q;

	sq = vector_norm(p);

	q.q0 = (-a * sq + f * sqrtf(f * f + (1 - a * a) * sq)) / (f * f + sq);

	inv = (a + q.q0) / f;
	q.q1 = p.x * inv;
	q.q2 = p.y * inv;
	q.q3 = p.z * inv;

	return q;
}

/*
 * @brief				Performs quaternion multiplication.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] a 		First quaternion to be multiplied.
 * @param[in] b 		Second quaternion to be multiplied.
 * @return 				Multiplied quaternion.
 */
static inline quaternion_t qmult(quaternion_t a, quaternion_t b)
{
	/**
	 * q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
     * 		a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
     * 		a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
     * 		a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)]
     * */
	quaternion_t r;

	r.q0 = a.q0 * b.q0 - a.q1 * b.q1 - a.q2 * b.q2 - a.q3 * b.q3;
	r.q1 = a.q0 * b.q1 + a.q1 * b.q0 + a.q2 * b.q3 - a.q3 * b.q2;
	r.q2 = a.q0 * b.q2 - a.q1 * b.q3 + a.q2 * b.q0 + a.q3 * b.q1;
	r.q3 = a.q0 * b.q3 + a.q1 * b.q2 - a.q2 * b.q1 + a.q3 * b.q0;

	return r;
}

/*
 * @brief				Conjugates a quaternion.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] q 		Quaternion to be conjugated.
 * @return 				Conjugated quaternion.
 */
static inline quaternion_t qconj(quaternion_t q)
{
	q.q1 = - q.q1;
	q.q2 = - q.q2;
	q.q3 = - q.q3;

	return q;
}

/*
 * @brief				Calculates the norm of a quaternion.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] q 		Input quaternion.
 * @return 				Quaternion norm.
 */
static inline float qnorm(quaternion_t q)
{
	return sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
}

/*
 * @brief				Performs quaternion normalization.
 *
 * @author				Emil Fresk, Luleå University of Technology
 * @param[in] q 		Quaternion to be normalized.
 * @return 				Normalized quaternion.
 */
static inline quaternion_t qnormalize(quaternion_t q)
{
	float invNorm = 1.0f / sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);

	q.q0 *= invNorm;
	q.q1 *= invNorm;
	q.q2 *= invNorm;
	q.q3 *= invNorm;

	return q;
}


/* Global function defines */
void euler2quat(float, float, float, quaternion_t *);
void q2dcm(float R[3][3], quaternion_t *q);

#endif
