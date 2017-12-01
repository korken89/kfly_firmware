/* *
 *
 * Abstraction Layer for RC Outputs
 *
 * */

#include "rc_output.h"
#include "flash_save.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

#define TIM4_UP_DMA_STREAM    STM32_DMA_STREAM_ID(1, 6)
#define TIM4_UP_DMA_CHANNEL   2
#define TIM8_UP_DMA_STREAM    STM32_DMA_STREAM_ID(2, 1)
#define TIM8_UP_DMA_CHANNEL   7
#define TIM_DMA_PRIO          3 // 0..3 (low..high)

#define PWM_FREQUENCY         1000000
#define PWM_50HZ_PERIOD       20000
#define PWM_400HZ_PERIOD      2500
#define PWM_BIAS              1000
#define PWM_GAIN              PWM_BIAS

#define ONESHOT125_FREQUENCY  8400000
#define ONESHOT125_PERIOD     2100
#define ONESHOT125_BIAS       (ONESHOT125_PERIOD / 2 - 1)
#define ONESHOT125_GAIN       ONESHOT125_BIAS

#define ONESHOT42_FREQUENCY   28000000
#define ONESHOT42_PERIOD      2352
#define ONESHOT42_BIAS        (ONESHOT42_PERIOD / 2 - 1)
#define ONESHOT42_GAIN        ONESHOT42_BIAS

#define MULTISHOT_FREQUENCY   84000000
#define MULTISHOT_PERIOD      2100
#define MULTISHOT_BIAS        420
#define MULTISHOT_GAIN        1680

#define DSHOT1200_FREQUENCY   84000000
#define DSHOT600_FREQUENCY    (DSHOT1200_FREQUENCY / 2)
#define DSHOT300_FREQUENCY    (DSHOT1200_FREQUENCY / 4)
#define DSHOT150_FREQUENCY    (DSHOT1200_FREQUENCY / 8)
#define DSHOT_PERIOD          70
#define DSHOT_BIAS            48
#define DSHOT_GAIN            1999
#define DSHOT_BIT_0           26
#define DSHOT_BIT_1           53

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables and types.                                         */
/*===========================================================================*/

const uint16_t rcoutput_period[] = {
    PWM_50HZ_PERIOD,    // 50HZ
    PWM_400HZ_PERIOD,   // 400HZ
    ONESHOT125_PERIOD,  // ONESHOT125
    ONESHOT42_PERIOD,   // ONESHOT42
    MULTISHOT_PERIOD,   // MULTISHOT
    DSHOT_PERIOD,       // DSHOT150
    DSHOT_PERIOD,       // DSHOT300
    DSHOT_PERIOD,       // DSHOT600
    DSHOT_PERIOD        // DSHOT1200
};

const uint32_t rcoutput_timer_frequency[] = {
    PWM_FREQUENCY,        // 50HZ
    PWM_FREQUENCY,        // 400HZ
    ONESHOT125_FREQUENCY, // ONESHOT125
    ONESHOT42_FREQUENCY,  // ONESHOT42
    MULTISHOT_FREQUENCY,  // MULTISHOT
    DSHOT150_FREQUENCY,   // DSHOT150
    DSHOT300_FREQUENCY,   // DSHOT300
    DSHOT600_FREQUENCY,   // DSHOT600
    DSHOT1200_FREQUENCY   // DSHOT1200
};

/**
 * @brief RC output channel lookup table for timer channel to output channel.
 */
const uint8_t rcoutput_channellut[] = {0, 1, 2, 3, 3, 2, 1, 0};

rcoutput_configuration_t rcoutput_config;

/**
 * @brief RC output bank settings.
 */
rcoutput_settings_t rcoutput_settings;

/**
 * @brief Working area for the RC output flash save thread.
 */
THD_WORKING_AREA(waThreadRCOutputFlashSave, 256);

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/**
 * @brief           Thread for the flash save operation.
 *
 * @param[in] arg   Unused.
 * @return          Unused.
 */
static THD_FUNCTION(ThreadRCOutputFlashSave, arg)
{
    UNUSED(arg);

    /* Event registration for new estimation */
    event_listener_t el;

    /* Set thread name */
    chRegSetThreadName("RCOutput FlashSave");

    /* Register to new estimation */
    chEvtRegisterMask(ptrGetFlashSaveEventSource(),
                      &el,
                      FLASHSAVE_SAVE_EVENTMASK);

    while (1)
    {
        /* Wait for new estimation */
        chEvtWaitOne(FLASHSAVE_SAVE_EVENTMASK);

        /* Save RC input settings to flash */
        FlashSave_Write(FlashSave_STR2ID("RCOT"),
                        true,
                        (uint8_t *)&rcoutput_settings,
                        RCOUTPUT_SETTINGS_SIZE);
    }
}

/**
 * @brief   Reset for the RC output settings structure.
 */
static void RCOutputSettingsReset(void)
{
    for (int i = 0; i < RCOUTPUT_NUM_OUTPUTS; i++)
        rcoutput_settings.channel_enabled[i] = false;

    rcoutput_settings.mode_bank1 = RCOUTPUT_MODE_DSHOT150;
    rcoutput_settings.mode_bank2 = RCOUTPUT_MODE_DSHOT150;
}

/**
 * @brief   Setups the DMAs for the timers.
 */
static void RCOutputDMAInit(void)
{
    // Set CHSEL bits according to DMA Channel
    // Set DIR bits according to Memory to peripheral direction
    // Set PINC bit according to DMA Peripheral Increment Disable
    // Set MINC bit according to DMA Memory Increment Enable
    // Set PSIZE bits according to Peripheral DataSize Word
    // Set MSIZE bits according to Memory DataSize Word
    // Set CIRC bit according to disable circular mode
    // Set PL bits according to very high priority
    // Set MBURST bits according to single memory burst
    // Set PBURST bits according to single peripheral burst
    const uint32_t dma_mode = STM32_DMA_CR_PL(TIM_DMA_PRIO)           |
                              STM32_DMA_CR_DIR_M2P                    |
                              STM32_DMA_CR_PSIZE_HWORD                |
                              STM32_DMA_CR_MSIZE_HWORD                |
                              STM32_DMA_CR_MINC                       |
                              STM32_DMA_CR_PBURST_SINGLE              |
                              STM32_DMA_CR_MBURST_SINGLE;

    // TIM4_UP DMA
    dmaStreamSetMemory0(rcoutput_config.bank1_dmasp,
                        rcoutput_config.bank1_buffer);
    dmaStreamSetPeripheral(rcoutput_config.bank1_dmasp,
                           &rcoutput_config.bank1_tim->DMAR);
    dmaStreamSetTransactionSize(rcoutput_config.bank1_dmasp, 0);
    dmaStreamSetMode(rcoutput_config.bank1_dmasp,
                     dma_mode | STM32_DMA_CR_CHSEL(TIM4_UP_DMA_CHANNEL));

    // TIM8_UP DMA
    dmaStreamSetMemory0(rcoutput_config.bank2_dmasp,
                        rcoutput_config.bank2_buffer);
    dmaStreamSetPeripheral(rcoutput_config.bank2_dmasp,
                           &rcoutput_config.bank2_tim->DMAR);
    dmaStreamSetTransactionSize(rcoutput_config.bank2_dmasp, 0);
    dmaStreamSetMode(rcoutput_config.bank2_dmasp,
                     dma_mode | STM32_DMA_CR_CHSEL(TIM8_UP_DMA_CHANNEL));

    // Set the end of the buffers to a "off" value
    for (int i = 0; i < 4; i++)
    {
      rcoutput_config.bank1_buffer[16][i] = 0;
      rcoutput_config.bank2_buffer[16][i] = 0;
    }
}

/**
 * @brief   Tells the DMAs to start transferring timer values to the timers.
 */
static void SendDMAPayload(uint32_t bank1_size, uint32_t bank2_size)
{
    // Reset DMA payload size, flags and enable it


    // Check so last transmission finished
    if ((rcoutput_config.bank1_dmasp->stream->CR & STM32_DMA_CR_EN) == 0)
    {
        dmaStreamSetTransactionSize(rcoutput_config.bank1_dmasp, bank1_size);

        // DMA1, Stream 6, Channel 4
        uint32_t flags = rcoutput_config.bank1_dmap->HISR;
        rcoutput_config.bank1_dmap->HIFCR = flags & (STM32_DMA_ISR_MASK << 16U);

        rcoutput_config.bank1_tim->EGR |= TIM_EGR_UG;
        dmaStreamEnable(rcoutput_config.bank1_dmasp);
    }


    // Check so last transmission finished
    if ((rcoutput_config.bank2_dmasp->stream->CR & STM32_DMA_CR_EN) == 0)
    {
        dmaStreamSetTransactionSize(rcoutput_config.bank2_dmasp, bank2_size);

        // DMA2, Stream 1, Channel 7
        uint32_t flags = rcoutput_config.bank2_dmap->LISR;
        rcoutput_config.bank2_dmap->LIFCR = flags & (STM32_DMA_ISR_MASK << 6U);

        rcoutput_config.bank2_tim->EGR |= TIM_EGR_UG;
        dmaStreamEnable(rcoutput_config.bank2_dmasp);

    }
}

/**
 * @brief   Takes a value [0 .. 2047] and a telemetry request and generates a
 *          Dshot data packet including checksum.
 */
static uint16_t PrepareDshotPacket(const uint16_t value, bool request_telemetry)
{
  uint16_t packet = (value << 1) | (request_telemetry ? 1 : 0);

  // compute checksum
  uint16_t csum = 0;
  uint16_t csum_data = packet;

  for (int i = 0; i < 3; i++)
  {
      csum ^= csum_data;
      csum_data >>= 4;
  }

  packet = (packet << 4) | (csum & 0xf);

  return packet;
}

/**
 * @brief   A generic helper to fill the timers' data buffers.
 */
void SetChannelWidthGeneric(rcoutput_mode_t mode, int idx, float value,
                            bool request_telemetry,
                            uint16_t buffer[RCOUTPUT_NUM_OUTPUTS * 2 + 1]
                                           [RCOUTPUT_BANK_SIZE])
{
  switch (mode)
  {
    case RCOUTPUT_MODE_50HZ_PWM:
    case RCOUTPUT_MODE_400HZ_PWM:
      buffer[0][idx] = value * PWM_GAIN + PWM_BIAS;
      break;

    case RCOUTPUT_MODE_ONESHOT125:
      buffer[0][idx] = value * ONESHOT125_GAIN + ONESHOT125_BIAS;
      buffer[1][idx] = 0;
      break;

    case RCOUTPUT_MODE_ONESHOT42:
      buffer[0][idx] = value * ONESHOT42_GAIN + ONESHOT42_BIAS;
      buffer[1][idx] = 0;
      break;

    case RCOUTPUT_MODE_MULTISHOT:
      buffer[0][idx] = value * MULTISHOT_GAIN + MULTISHOT_BIAS;
      buffer[1][idx] = 0;
      break;

    case RCOUTPUT_MODE_DSHOT150:
    case RCOUTPUT_MODE_DSHOT300:
    case RCOUTPUT_MODE_DSHOT600:
    case RCOUTPUT_MODE_DSHOT1200:
    {
      uint16_t packet;

      if (value == 0.0f)
        packet = PrepareDshotPacket(0, request_telemetry);
      else
        packet = PrepareDshotPacket(value * DSHOT_GAIN + DSHOT_BIAS,
                                    request_telemetry);

      // Convert packet to times
      for (int i = 0; i < 16; i++)
      {
        buffer[i][idx] = (packet & 0x8000) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        packet <<= 1;
      }

      buffer[16][idx] = 0;
    }

    default:
      return;

  }
}



/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Initializes RC outputs' settings and data structures.
 */
void RCOutputInit(void)
{
    /* Reset the settings structure. */
    RCOutputSettingsReset();

    /* Read RC output settings from flash */
    FlashSave_Read(FlashSave_STR2ID("RCOT"),
                   (uint8_t *)&rcoutput_settings,
                   RCOUTPUT_SETTINGS_SIZE);

    /* Start the Flash Save thread */
    chThdCreateStatic(waThreadRCOutputFlashSave,
                      sizeof(waThreadRCOutputFlashSave),
                      NORMALPRIO,
                      ThreadRCOutputFlashSave,
                      NULL);

    // Set up DMAs and initialize timers
    rcoutput_config.bank1_dmap = DMA1;
    rcoutput_config.bank1_dmasp = STM32_DMA_STREAM(TIM4_UP_DMA_STREAM);

    if (dmaStreamAllocate(rcoutput_config.bank1_dmasp, 10, NULL, NULL))
    {
        // Error, already taken.
        osalSysHalt("rcoutput stream taken");
    }

    rcoutput_config.bank2_dmap = DMA2;
    rcoutput_config.bank2_dmasp = STM32_DMA_STREAM(TIM8_UP_DMA_STREAM);

    if (dmaStreamAllocate(rcoutput_config.bank2_dmasp, 10, NULL, NULL))
    {
        // Error, already taken.
        osalSysHalt("rcoutput stream taken");
    }

    rcoutput_config.bank1_tim = STM32_TIM4;
    rcoutput_config.bank2_tim = STM32_TIM8;

    rcoutput_config.bank1_tim_clock = STM32_TIMCLK1;
    rcoutput_config.bank2_tim_clock = STM32_TIMCLK2;

    RCOutputDMAInit();

    RCOutputTimerInit();
}

/**
 * @brief   Initializes or reinitialized RC outputs based on the settings.
 */
void RCOutputTimerInit(void)
{
    //
    // Timer configuration
    //
    stm32_tim_t *tim_b1 = rcoutput_config.bank1_tim;
    stm32_tim_t *tim_b2 = rcoutput_config.bank2_tim;

    rccEnableTIM4(FALSE);
    rccResetTIM4();

    rccEnableTIM8(FALSE);
    rccResetTIM8();


    //
    // Reset settings and count
    //
    tim_b1->CR1 = 0;
    tim_b1->CR2 = 0;
    tim_b1->CNT = 0;
    tim_b1->CCR[0] = 0;
    tim_b1->CCR[1] = 0;
    tim_b1->CCR[2] = 0;
    tim_b1->CCR[3] = 0;
    tim_b1->CCER  = 0;

    tim_b2->CR1 = 0;
    tim_b2->CR2 = 0;
    tim_b2->CNT = 0;
    tim_b2->CCR[0] = 0;
    tim_b2->CCR[1] = 0;
    tim_b2->CCR[2] = 0;
    tim_b2->CCR[3] = 0;
    tim_b2->CCER  = 0;

    //
    // Set up clock settings
    //
    tim_b1->ARR = rcoutput_period[rcoutput_settings.mode_bank1] - 1;
    tim_b1->PSC = rcoutput_config.bank1_tim_clock /
                      rcoutput_timer_frequency[rcoutput_settings.mode_bank1] -
                  1;

    tim_b2->ARR = rcoutput_period[rcoutput_settings.mode_bank2] - 1;
    tim_b2->PSC = rcoutput_config.bank2_tim_clock /
                      rcoutput_timer_frequency[rcoutput_settings.mode_bank2] -
                  1;

    //
    // Output settings
    //

    // Preload enable and select output compare mode 1
    tim_b1->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE |
                    STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;
    tim_b1->CCMR2 = STM32_TIM_CCMR2_OC3M(6) | STM32_TIM_CCMR2_OC3PE |
                    STM32_TIM_CCMR2_OC4M(6) | STM32_TIM_CCMR2_OC4PE;

    tim_b2->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE |
                    STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;
    tim_b2->CCMR2 = STM32_TIM_CCMR2_OC3M(6) | STM32_TIM_CCMR2_OC3PE |
                    STM32_TIM_CCMR2_OC4M(6) | STM32_TIM_CCMR2_OC4PE;

    // Enable the TIM Main Output (for advanced control timers)
    tim_b2->BDTR  = STM32_TIM_BDTR_MOE;

    // Enable outputs (if enabled in settings)
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_1]])
      tim_b1->CCER |= STM32_TIM_CCER_CC1E;
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_2]])
      tim_b1->CCER |= STM32_TIM_CCER_CC2E;
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_3]])
      tim_b1->CCER |= STM32_TIM_CCER_CC3E;
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_4]])
      tim_b1->CCER |= STM32_TIM_CCER_CC4E;

    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_5] + 4])
      tim_b2->CCER |= STM32_TIM_CCER_CC1E;
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_6] + 4])
      tim_b2->CCER |= STM32_TIM_CCER_CC2E;
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_7] + 4])
      tim_b2->CCER |= STM32_TIM_CCER_CC3E;
    if (rcoutput_settings
            .channel_enabled[rcoutput_channellut[RCOUTPUT_CHANNEL_8] + 4])
      tim_b2->CCER |= STM32_TIM_CCER_CC4E;

    //
    // Setup Timer DMA settings
    //

    // Configure of the DMA Base register to CCR1 and the DMA Burst Length to 4
    tim_b1->DCR = STM32_TIM_DCR_DBA(13) | STM32_TIM_DCR_DBL(3);
    tim_b2->DCR = STM32_TIM_DCR_DBA(13) | STM32_TIM_DCR_DBL(3);

    // TIM1 DMA Update enable
    tim_b1->DIER = TIM_DIER_UDE;
    tim_b2->DIER = TIM_DIER_UDE;

    // Enable the TIM Counter
    tim_b1->CR1 = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
    tim_b2->CR1 = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
}

/**
 * @brief   Disables all outputs, used when force disable of outputs is needed.
 */
void RCOutputDisableI(void)
{
    stm32_tim_t *tim_b1 = rcoutput_config.bank1_tim;
    stm32_tim_t *tim_b2 = rcoutput_config.bank2_tim;

    // Disable timer
    tim_b1->CR1 = 0;
    tim_b1->CR2 = 0;
    tim_b1->CNT = 0;

    tim_b2->CR1 = 0;
    tim_b2->CR2 = 0;
    tim_b2->CNT = 0;

    // Disable outputs
    tim_b1->CCER = 0;
    tim_b2->CCER = 0;
}

/**
 * @brief               Set relative output channel width, but does not apply.
 * @details             Set the width using 0% to 100%, but a call to
 *                      @p RCOutputSync must be made to commit new values to
 *                      the timers.
 *
 * @param[in] channel   Channel selector.
 * @param[in] width     New width in 0.0 to 1.0.
 */
void RCOutputSetChannelWidth(const rcoutput_channel_t channel,
                             float value)
{
    const int idx = rcoutput_channellut[channel];

    // Bound input value
    if (value > 1.0f)
        value = 1.0f;
    else if (value < 0.0f)
        value = 0.0f;

    if (channel <= RCOUTPUT_CHANNEL_4)
    {
        SetChannelWidthGeneric(rcoutput_settings.mode_bank1,
                               idx,
                               value,
                               rcoutput_config.request_telemetry[idx],
                               rcoutput_config.bank1_buffer);
    }
    else
    {
        SetChannelWidthGeneric(rcoutput_settings.mode_bank2,
                               idx,
                               value,
                               rcoutput_config.request_telemetry[idx + 4],
                               rcoutput_config.bank2_buffer);
    }
}

/**
 * @brief   Moves all channel widths into the timers.
 */
void RCOutputSync(void)
{
    uint32_t bank1_payload_size;
    uint32_t bank2_payload_size;

    if (rcoutput_settings.mode_bank1 < RCOUTPUT_MODE_ONESHOT_START)
        bank1_payload_size = 1;
    else if (rcoutput_settings.mode_bank1 < RCOUTPUT_MODE_DSHOT_START)
        bank1_payload_size = 2;
    else
        bank1_payload_size = 17;

    if (rcoutput_settings.mode_bank2 < RCOUTPUT_MODE_ONESHOT_START)
        bank2_payload_size = 1;
    else if (rcoutput_settings.mode_bank2 < RCOUTPUT_MODE_DSHOT_START)
        bank2_payload_size = 2;
    else
        bank2_payload_size = 17;

    SendDMAPayload(bank1_payload_size * 4, bank2_payload_size * 4);
}

/**
 * @brief     Checks if the effects of the @p RCOutputSync have taken hold.
 *
 * @return    True if there is an Sync active, false if sync has finished.
 */
bool RCOutputSyncActive(void)
{
  if ((rcoutput_config.bank1_dmasp->stream->CR & STM32_DMA_CR_EN) != 0 ||
      (rcoutput_config.bank2_dmasp->stream->CR & STM32_DMA_CR_EN) != 0)
    return true;
  else
    return false;
}

/**
 * @brief               Parses a payload from the serial communication for
 *                      all the RC output settings.
 *
 * @param[in] payload   Pointer to the payload location.
 * @param[in] size      Size of the payload.
 */
void vParseSetRCOutputSettings(const uint8_t *payload,
                               const size_t data_length)
{
    if (data_length == RCOUTPUT_SETTINGS_SIZE)
    {
        osalSysLock();

        /* Save the data */
        memcpy((uint8_t *)ptrGetRCOutoutSettings(),
               payload,
               RCOUTPUT_SETTINGS_SIZE);

        osalSysUnlock();
    }

    /* Reinitialize the RC Output module. */
    RCOutputTimerInit();
}

/**
 * @brief           Return the pointer to the RC Input settings.
 *
 * @return          Pointer to the RC input settings.
 */
rcoutput_settings_t *ptrGetRCOutoutSettings(void)
{
    return &rcoutput_settings;
}
