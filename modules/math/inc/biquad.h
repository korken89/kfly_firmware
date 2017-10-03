
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
  biquad_df2t_state_t state;
  biquad_coeffs_t coeffs;
} biquad_df2t_t;


/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

static inline void BiquadInitStateDF2T(biquad_df2t_state_t *p)
{
  p->s1 = 0;
  p->s2 = 0;
}

static inline float BiquadDF2TApply(biquad_df2t_t *p, float in)
{
    float out;

    out         = p->state.s1 + p->coeffs.b0 * in;
    p->state.s1 = p->state.s2 + p->coeffs.b1 * in - p->coeffs.a1 * out;
    p->state.s2 =               p->coeffs.b2 * in - p->coeffs.a2 * out;

    return out;
}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void BiquadInitCoeffs(biquad_coeffs_t *coeffs,
                      float sampling_frequency,
                      float center_frequency,
                      float Q,
                      biquad_type_t type);


#endif
