#ifndef __TRIGONOMETRY_H
#define __TRIGONOMETRY_H

#include <math.h>
#include <stdint.h>

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/
#define PI ( 3.14159265359f )

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/*
 * @brief               Fast atan2 implementation.
 *
 * @param[in] y         First parameter.
 * @param[in] x         Second parameter.
 * @return              The acrtan.
 */
static inline float fastatan2(const float y, const float x)
{
    const float coeff_1 = PI * 0.25f;
    const float coeff_2 = PI * 0.75f;
    float abs_y = fabsf(y);
    float angle, r;

    if (x >= 0.0f)
    {
        r = (x + abs_y);

        if (fabsf(r) < 1.0e-5)
            return 0.0f;

        r = (x - abs_y) / r;
        angle = coeff_1 - coeff_1 * r;
    }
    else
    {
        r = (abs_y - x);

        if (fabsf(r) < 1.0e-5)
            return 0.0f;

        r = (x + abs_y) / r;
        angle = coeff_2 - coeff_1 * r;
    }

    return y < 0.0f ? -angle : angle;
}

/*
 * @brief               Fast 2^x implementation.
 *
 * @param[in] p         Input exponent.
 * @return              The calculated power.
 */
static inline float fastpow2(const float p)
{
    float offset = (p < 0) ? 1.0f : 0.0f;
    float clipp = (p < -126) ? -126.0f : p;
    int w = clipp;
    float z = clipp - w + offset;
    union { uint32_t i; float f; } v;
    v.i = (uint32_t)((1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z));

    return v.f;
}


/*
 * @brief               Fast e^x implementation.
 *
 * @param[in] p         Input exponent.
 * @return              The calculated power.
 */
static inline float fastexp(const float p)
{
    return fastpow2(1.442695040f * p);
}


/*
 * @brief               Signal bounding implementation. Bounds value to be
 *                      within the min and max.
 *
 * @param[in] max       Maximum limit.
 * @param[in] min       Minimum limit.
 * @param[in] x         Input value.
 * @return              The bounded value.
 */
static inline float bound(const float max, const float min, const float x)
{
    if (x > max)
        return max;
    else if (x < min)
        return min;
    else
        return x;
}


/*
 * @brief               Fast sine approximation
 *
 * @param[in] x         Input value.
 * @return              The calculated sine value.
 */
static inline float fast_sin(float x)
{
    if (x < -PI)
        x += 2.0f * PI;
    else if (x > PI)
        x -= 2.0f * PI;

    const float B = 4.0f/PI;
    const float C = -4.0f/(PI*PI);
    const float P = 0.225f;

    float y = B * x + C * x * fabsf(x);
    y = P * (y * fabsf(y) - y) + y;

    return y;
}

/*
 * @brief               Fast cosine approximation
 *
 * @param[in] x         Input value.
 * @return              The calculated cosine value.
 */
static inline float fast_cos(float x)
{
    x += PI*0.5f;

    if (x < -PI)
        x += 2.0f * PI;
    else if (x > PI)
        x -= 2.0f * PI;

    const float B = 4.0f/PI;
    const float C = -4.0f/(PI*PI);
    const float P = 0.225f;

    float y = B * x + C * x * fabsf(x);
    y = P * (y * fabsf(y) - y) + y;

    return y;
}

/*
 * @brief               Evaluation of a polynomial using Horner's Rule.
 * @details             Given coeffs = [b0 b1 ... bN] the polynomial's
 *                      evaluation becomes:
 *                      y = b0 + x * (b1 + x * (b2 ... + x * (bN-1 + x * bN))),
 *                      which in code is evaluated backwards.
 *
 * @param[in] x         Value to evaluate the polynomial at.
 * @param[in] coeffs    Array containing the coefficients.
 * @param[in] size      Size of the coefficients array.
 * @return              The evaluated polynomical.
 */
static inline float polyeval_horner(const float x,
                                    const float *coeffs,
                                    const size_t size)
{
    int i;

    /* Load the last coefficient to not waste one multiplication. */
    float accumulator = coeffs[size - 1];

    /* Run for the size of the polynomial. */
    for (i = size - 2; i >= 0; i--)
        accumulator = accumulator * x + coeffs[i];

    return accumulator;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif
