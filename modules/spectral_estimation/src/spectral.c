
#include <stdint.h>
#include "ch.h"
#include "hal.h"
#include "spectral.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"

void stage_rfft_f32(arm_rfft_fast_instance_f32 * S, float32_t * p, float32_t * pOut);
void arm_cfft_radix8by2_f32( arm_cfft_instance_f32 * S, float32_t * p1);
void arm_cfft_radix8by4_f32( arm_cfft_instance_f32 * S, float32_t * p1);
void arm_radix8_butterfly_f32(float32_t * pSrc, uint16_t fftLen, const float32_t * pCoef, uint16_t twidCoefModifier);
void arm_bitreversal_32(uint32_t * pSrc, const uint16_t bitRevLen, const uint16_t * pBitRevTable);


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
