#include <stdint.h>
#include "spectral.h"
#include "hann_windows.h"
#include "arm_common_tables.h"
#include "sensor_read.h"

// Functions hidden in the CMSIS DSP library that are needed
void stage_rfft_f32(arm_rfft_fast_instance_f32 *S, float32_t *p,
                    float32_t *pOut);
void arm_cfft_radix8by2_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_cfft_radix8by4_f32(arm_cfft_instance_f32 *S, float32_t *p1);
void arm_radix8_butterfly_f32(float32_t *pSrc, uint16_t fftLen,
                              const float32_t *pCoef,
                              uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen,
                        const uint16_t *pBitRevTable);




void ApplyFFT(arm_rfft_fast_instance_f32 *fft, float *input, float *scratch)
{
  arm_cfft_instance_f32 *Sint  = &(fft->Sint);

#if SPECTRAL_FFT_SIZE == 32 || SPECTRAL_FFT_SIZE == 256 || SPECTRAL_FFT_SIZE == 2048
  arm_cfft_radix8by2_f32(Sint, input);
#elif SPECTRAL_FFT_SIZE == 64 || SPECTRAL_FFT_SIZE == 512 || SPECTRAL_FFT_SIZE == 4096
  arm_cfft_radix8by4_f32(Sint, input);
#elif SPECTRAL_FFT_SIZE == 128 || SPECTRAL_FFT_SIZE == 1024 || SPECTRAL_FFT_SIZE == 8192
  arm_radix8_butterfly_f32(input, SPECTRAL_FFT_SIZE / 2, Sint->pTwiddle, 1);
#else
#error "SPECTRAL_FFT_SIZE is not a valid number, must be a power of 2"
#endif

  arm_bitreversal_32((uint32_t*) input, Sint->bitRevLength, Sint->pBitRevTable);
  stage_rfft_f32(fft, input, scratch);
}



void EstimateVibrationFrequencies(spectral_estimation_t *p)
{
  // Calculate the magnitude squared of the FFT
  arm_cmplx_mag_squared_f32(p->scratchpad, p->fft_area, SPECTRAL_FFT_SIZE/2);

  // Find the frequency of the FFT peak by averaging
  const int idx_start = 2;
  const int idx_end = 7;

  float sum = 0;
  float weighted_sum = 0;

  for (int i = idx_start; i < idx_end; i++)
  {
    weighted_sum += i * p->fft_area[i];
    sum += p->fft_area[i];
  }

  if (sum < 1e-6f)
    return;


  float frequency_estimate = weighted_sum / sum;

}

void SpectralEstimationInit(spectral_estimation_t *p)
{
  arm_cfft_instance_f32 *Sint  = &(p->fft_instance.Sint);

  // Lengths
  Sint->fftLen                 = SPECTRAL_FFT_SIZE / 2;
  p->fft_instance.fftLenRFFT   = SPECTRAL_FFT_SIZE;

  // Lookup tables
#if SPECTRAL_FFT_SIZE == 32
  Sint->bitRevLength           = ARMBITREVINDEXTABLE__16_TABLE_LENGTH;
  Sint->pBitRevTable           = (uint16_t *)armBitRevIndexTable16;
  Sint->pTwiddle               = (float32_t *)twiddleCoef_16;
  p->fft_instance.pTwiddleRFFT = (float32_t *)twiddleCoef_rfft_32;
#elif SPECTRAL_FFT_SIZE == 64
  Sint->bitRevLength           = ARMBITREVINDEXTABLE__32_TABLE_LENGTH;
  Sint->pBitRevTable           = (uint16_t *)armBitRevIndexTable32;
  Sint->pTwiddle               = (float32_t *)twiddleCoef_32;
  p->fft_instance.pTwiddleRFFT = (float32_t *)twiddleCoef_rfft_64;
#elif SPECTRAL_FFT_SIZE == 128
  Sint->bitRevLength           = ARMBITREVINDEXTABLE__64_TABLE_LENGTH;
  Sint->pBitRevTable           = (uint16_t *)armBitRevIndexTable64;
  Sint->pTwiddle               = (float32_t *)twiddleCoef_64;
  p->fft_instance.pTwiddleRFFT = (float32_t *)twiddleCoef_rfft_128;
#else
#error "SPECTRAL_FFT_SIZE is not valid, only [32, 64, 128] are supported"
#endif

  // Init sample vectors
  for (int i = 0; i < SPECTRAL_FFT_SIZE; i++)
  {
    p->samples_x[i] = 0;
    p->samples_y[i] = 0;
    p->samples_z[i] = 0;
  }

  // Init filters
  for (int i = 0; i < 3; i++)
  {
    BiquadInitStateDF2T(&p->frequency_filters[i].state);
    BiquadUpdateCoeffs(&p->frequency_filters[i].coeffs,
                       SENSOR_ACCGYRO_HZ / 3.0f,
                       SPECTRAL_FILTERS_CUTOFF,
                       ACCGYRO_BUTTERWORTH_Q,
                       BIQUAD_TYPE_LPF);
  }

  p->axis_counts = 0;
  p->state = SPECTRAL_X_AXIS;
}



void SpectralEstimationUpdate(spectral_estimation_t *p, float x, float y, float z)
{
  // Apply Hann window and save for processing
  const float hann = hann_window[p->axis_counts];
  p->samples_x[p->axis_counts] = x * hann;
  p->samples_y[p->axis_counts] = y * hann;
  p->samples_z[p->axis_counts] = z * hann;

  // Increment counters
  p->axis_counts = (p->axis_counts + 1) % SPECTRAL_FFT_SIZE;


  // Check which axis to perform FFT on
  if (p->state == SPECTRAL_X_AXIS)
  {
    // X axis FFT
    for (int i = 0; i < SPECTRAL_FFT_SIZE; i++)
      p->fft_area[i] = p->samples_x[i];
  }
  else if (p->state == SPECTRAL_Y_AXIS)
  {
    // Y axis FFT
    for (int i = 0; i < SPECTRAL_FFT_SIZE; i++)
      p->fft_area[i] = p->samples_y[i];
  }
  else
  {
    // Z axis FFT
    for (int i = 0; i < SPECTRAL_FFT_SIZE; i++)
      p->fft_area[i] = p->samples_y[i];
  }

  // This makes the FFT run on one axis per input sample, so the
  // FFT is practically run on ever 3rd sample on each axis
  ApplyFFT(&p->fft_instance, p->fft_area, p->scratchpad);

  // Final processing step, estimate the vibration peak
  EstimateVibrationFrequencies(p);

  // Update state
  if (p->state == SPECTRAL_X_AXIS)
    p->state = SPECTRAL_Y_AXIS;
  else if (p->state == SPECTRAL_Y_AXIS)
    p->state = SPECTRAL_Z_AXIS;
  else
    p->state = SPECTRAL_X_AXIS;
}




float rfft_data[32] = {
 -30,   772,   993,    72,  -803,  -871,  -236,   747,  1040,   372,  -595,  -920,  -604,   539,  1070,   679,  -288,  -925,  -398,   139,   757,  1028,    52,  -911,  -644,   -77,   792,   879,   164,  -840, -1030,  -509//,   616,   968,   706,  -277,  -975,  -672,   412,   849,   812,  -226,  -957,  -899,    75,   773,   795,   179,  -811,  -918,  -298,   697,   730,   443,  -373,  -896,  -430,   412,  1011,   661,  -295, -1049,  -774,   150
};
float rfft_data2[32];

volatile uint32_t fft_cyc1;
volatile uint32_t fft_cyc2;

void test_spectral(void)
{
  //arm_rfft_fast_init_f32(&fftInstance, fftSize); // Instantiates all twiddle tables (~100kB)

  uint32_t fftSize = 32;
  arm_rfft_fast_instance_f32 fftInstance;
  arm_cfft_instance_f32 *Sint = &(fftInstance.Sint);

  Sint->fftLen                = fftSize / 2;
  fftInstance.fftLenRFFT      = fftSize;
  Sint->bitRevLength          = ARMBITREVINDEXTABLE__16_TABLE_LENGTH;
  Sint->pBitRevTable          = (uint16_t *)armBitRevIndexTable16;
  Sint->pTwiddle              = (float32_t *)twiddleCoef_16;
  fftInstance.pTwiddleRFFT    = (float32_t *)twiddleCoef_rfft_32;

  fft_cyc1 = DWT->CYCCNT;
  arm_cfft_radix8by2_f32(Sint, rfft_data);
  arm_bitreversal_32((uint32_t*) rfft_data, Sint->bitRevLength, Sint->pBitRevTable);
  stage_rfft_f32(&fftInstance, rfft_data, rfft_data2);
  fft_cyc1 = DWT->CYCCNT - fft_cyc1;

  /* Process the data through the CFFT/CIFFT module */
  fft_cyc2 = DWT->CYCCNT;
  fft_cyc2 = DWT->CYCCNT - fft_cyc2;
}
