#ifndef __TRIGONOMETRY_H
#define __TRIGONOMETRY_H

/* Standard includes */
#include <math.h>
#include <stdint.h>

/* System includes */

/* Scheduler includes */

/* KFly includes */

/* Includes */

/* Defines */
#define PI ( 3.14159265359f )

/* Typedefs */

/* Global variable defines */

/* Global function defines */

/* Includes */

/* Private Defines */

/* Private Typedefs */

/* Global variable defines */

/* Inline functions */
static inline float fastatan2(float y, float x)
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

static inline float fastpow2(float p)
{
    float offset = (p < 0) ? 1.0f : 0.0f;
    float clipp = (p < -126) ? -126.0f : p;
    int w = clipp;
    float z = clipp - w + offset;
    union { uint32_t i; float f; } v;
    v.i = (uint32_t)((1 << 23) * (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z));

    return v.f;
}

static inline float fastexp(float p)
{
    return fastpow2(1.442695040f * p);
}

static inline float bound(float max, float min, float x)
{
    if (x > max)
        return max;
    else if (x < min)
        return min;
    else
        return x;
}

/*
 * fast_sin - Sine approximation
 * @param[in] x     Angle [rad]
 * @param[out] y    Sine
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

static inline float myfloor(float x)
{
    if (x > 0.0f)
        return (int)x;
    else
        return (int)(x - 0.9999999999999999f);
}

static inline float myfmodf(float x, float m)
{
    return x - myfloor(x / m) * m;
}

/* Private function defines */

/* Global function defines */

#endif
