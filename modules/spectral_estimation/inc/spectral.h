#ifndef __SPECTRAL_H
#define __SPECTRAL_H

#include "ch.h"
#include "hal.h"
#include "kfly_defs.h"
#include "arm_math.h"
#include "biquad.h"

/*===========================================================================*/
/* Module global definitions.                                                */
/*===========================================================================*/

#define SPECTRAL_FFT_SIZE                 32
#define SPECTRAL_FILTERS_CUTOFF           60

/*===========================================================================*/
/* Module data structures and types.                                         */
/*===========================================================================*/

typedef enum
{
  SPECTRAL_X_AXIS = 0,
  SPECTRAL_Y_AXIS,
  SPECTRAL_Z_AXIS
} spectral_estimation_state_t;

/**
 * @brief
 */
typedef struct
{
  float samples_x[SPECTRAL_FFT_SIZE];
  float samples_y[SPECTRAL_FFT_SIZE];
  float samples_z[SPECTRAL_FFT_SIZE];

  float fft_area[SPECTRAL_FFT_SIZE];
  float scratchpad[SPECTRAL_FFT_SIZE];

  biquad_df2t_t frequency_filters[3];

  arm_rfft_fast_instance_f32 fft_instance;
  uint32_t axis_counts;
  spectral_estimation_state_t state;
} spectral_estimation_t;


/*===========================================================================*/
/* Module macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
void SpectralEstimationInit(spectral_estimation_t *p);
void SpectralEstimationUpdate(spectral_estimation_t *p, float x, float y, float z);
void test_spectral(void);

/*===========================================================================*/
/* Module inline functions.                                                  */
/*===========================================================================*/

#endif
