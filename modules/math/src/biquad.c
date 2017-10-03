
#include "biquad.h"

/**
 * @brief
 */
void BiquadUpdateCoeffs(biquad_coeffs_t *coeffs,
                        float sampling_frequency,
                        float center_frequency,
                        float Q,
                        biquad_type_t type)
{
    // Based on:
    // http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html

    const float omega = 2.0f * M_PI * center_frequency / sampling_frequency;
    const float os    = sinf(omega);
    const float oc    = cosf(omega);
    const float alpha = os / (2.0f * Q);

    float b0 = 0;
    float b1 = 0;
    float b2 = 0;
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;

    switch (type)
    {
        case BIQUAD_LPF:
            b0 = (1.0f - oc) * 0.5f;
            b1 =  1.0f - oc;
            b2 = (1.0f - oc) * 0.5f;
            a0 =  1.0f + alpha;
            a1 = -2.0f * oc;
            a2 =  1.0f - alpha;
            break;

        case BIQUAD_HPF:
            b0 =  (1.0f + oc) * 0.5f;
            b1 = -(1.0f + oc);
            b2 =  (1.0f + oc) * 0.5f;
            a0 =   1.0f + alpha;
            a1 =  -2.0f * oc;
            a2 =   1.0f - alpha;
            break;

        case BIQUAD_NOTCH:
            b0 =  1.0f;
            b1 = -2.0f * oc;
            b2 =  1.0f;
            a0 =  1.0f + alpha;
            a1 = -2.0f * oc;
            a2 =  1.0f - alpha;
            break;
    }

    coeffs->b0 = b0 / a0;
    coeffs->b1 = b1 / a0;
    coeffs->b2 = b2 / a0;
    coeffs->a1 = a1 / a0;
    coeffs->a2 = a2 / a0;
}

