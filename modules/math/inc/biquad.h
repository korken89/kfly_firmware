
#ifndef __BIQUAD_H
#define __BIQUAD_H

#include <math.h>

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief
 */
typedef enum
{
    BIQUAD_LPF,
    BIQUAD_HPF,
    BIQUAD_NOTCH
} biquad_type_t;

/**
 * @brief
 */
typedef struct
{
  float y1; // internal state (Direct Form 1)
  float y2;
  float x1;
  float x2;
} biquad_df1_state_t;

/**
 * @brief
 */
typedef struct
{
  float s1; // internal state (Direct Form 2 Transposed)
  float s2;
} biquad_df2t_state_t;

/**
 * @brief
 */
typedef struct
{
  float a1; // denominator coefficients
  float a2;
  float b0; // nominator coefficients
  float b1;
  float b2;
} biquad_coeffs_t;

/**
 * @brief
 */
typedef struct
{
  biquad_df1_state_t state;
  biquad_coeffs_t coeffs;
} biquad_df1_t;

/**
 * @brief
 */
typedef struct
{
  biquad_df2t_state_t state;
  biquad_coeffs_t coeffs;
} biquad_df2t_t;


/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

/**
 * @brief
 */
static inline void BiquadInitStateDF1(biquad_df1_state_t *p)
{
  p->y1 = 0;
  p->y2 = 0;
  p->x1 = 0;
  p->x2 = 0;
}

/**
 * @brief
 */
static inline void BiquadInitStateDF2T(biquad_df2t_state_t *p)
{
  p->s1 = 0;
  p->s2 = 0;
}

/**
 * @brief
 */
static inline float BiquadDF2TApply(biquad_df2t_t *p, float in)
{
    float out;

    out         = p->state.s1 + p->coeffs.b0 * in;
    p->state.s1 = p->state.s2 + p->coeffs.b1 * in - p->coeffs.a1 * out;
    p->state.s2 =               p->coeffs.b2 * in - p->coeffs.a2 * out;

    return out;
}

/**
 * @brief
 */
static inline float BiquadDF1Apply(biquad_df1_t *p, float in)
{
    float out =   p->coeffs.b0 * in
                + p->coeffs.b1 * p->state.x1
                + p->coeffs.b2 * p->state.x2
                - p->coeffs.a1 * p->state.y1
                - p->coeffs.a2 * p->state.y2;

    p->state.x2 = p->state.x1;
    p->state.x1 = in;

    p->state.y2 = p->state.y1;
    p->state.y1 = out;

    return out;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void BiquadUpdateCoeffs(biquad_coeffs_t *coeffs,
                        float sampling_frequency,
                        float center_frequency,
                        float Q,
                        biquad_type_t type);


#endif
