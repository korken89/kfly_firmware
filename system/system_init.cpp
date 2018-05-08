//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "stm32f7xx.h"

//
// @brief   Init the BSS segment.
//
inline void bss_init()
{
  extern uint32_t __bss_start, __bss_end;

  uint32_t *from = &__bss_start;
  uint32_t *to = &__bss_end;

  while (from < to)
    *(from++) = 0;
}

//
// @brief   Init the data segment.
//
inline void data_init()
{
  extern uint32_t __text_end, __data_start, __data_end;

  uint32_t *to = &__data_start;
  const uint32_t *from = &__text_end;
  uint32_t size = ((uint32_t)&__data_end - (uint32_t)&__data_start) / 4;

  while (size--)
    *(to++) = *(from++);
}

//
// @brief   Run global constructors.
//
inline void constructor_init()
{
  /* Start and end points of the constructor list. */
  extern void (*__init_array_start)();
  extern void (*__init_array_end)();

  /* Call each function in the list. */
  for (void (**p)() = &__init_array_start; p < &__init_array_end; ++p)
  {
    (*p)();
  }
}

//
// @brief   Initialize the system caches
//
inline void InitCachesAndAccelerators()
{
  //
  // Enable I-Cache
  //
  SCB_EnableICache();

  //
  // Enable D-Cache
  //
  SCB_EnableDCache();

  //
  // Enable ART Accelerator
  //
  FLASH->ACR |= FLASH_ACR_ARTEN;

  //
  // Enable Flash Prefetch Buffer
  //
  FLASH->ACR |= FLASH_ACR_PRFTEN;
}

//
// @brief   Initialize the system clock and start the PLL.
//
inline void InitClocks()
{
  //
  // Enable power control clocks
  //
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  //
  // Configure internal voltage regulator
  //
  PWR->CR1 |= PWR_CR1_VOS;  // Voltage Scale 1

  //
  // Initialize PLL and Clock source
  //
  // Set HSE configuration to BYPASS (25 MHz oscillator)
  RCC->CR |= RCC_CR_HSEBYP;
  RCC->CR |= RCC_CR_HSEON;

  // Wait till HSE is ready
  while ((RCC->CR & RCC_CR_HSERDY) == 0)
    ;

  // ////////////////////////////////////////////////////////////////////
  // PLL startup sequence
  // ////////////////////////////////////////////////////////////////////

  // Disable PLL
  RCC->CR &= ~RCC_CR_PLLON;

  // Wait till PLL is ready
  while ((RCC->CR & RCC_CR_PLLRDY) != 0)
    ;

  // Configure main PLL
  RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE |         // HSE as source
                 (25 << RCC_PLLCFGR_PLLM_Pos) |   // PLLM (25 Mhz / 25 = 1 MHz)
                 (432 << RCC_PLLCFGR_PLLN_Pos) |  // PLLN (1 * 432 = 432 MHz)
                 (0 << RCC_PLLCFGR_PLLP_Pos) |    // PLLP (432 / 2 = 216 MHz)
                 (9 << RCC_PLLCFGR_PLLQ_Pos);     // PLLQ (432 / 9 = 48 MHz)

  // Enable the main PLL
  RCC->CR |= RCC_CR_PLLON;

  // Wait till PLL is ready
  while ((RCC->CR & RCC_CR_PLLRDY) == 0)
    ;

  //
  // Enable Over-drive mode
  //
  PWR->CR1 |= PWR_CR1_ODEN;
  while ((PWR->CSR1 & PWR_CSR1_ODRDY) == 0)
    ;

  //
  // Set flash latency (7 WS, taken from STM32CubeMX)
  //
  FLASH->ACR |= FLASH_ACR_LATENCY_7WS;
  while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_7WS)
    ;

  //
  // Initialize CPU, AHB and APB clocks
  //

  // Set highest PCLK dividers to not pass through high clock speeds
  RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16);  // PCLK1
  RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV16);  // PLCK2

  // Set HCLK divider
  RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_DIV1);

  // Set SYSCLK to PLL
  RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL);

  // Wait for PLL
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    ;

  // PCLK1 (APB1) config
  RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV4);

  // PCLK2 (APB2) config
  RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV2);

  // ////////////////////////////////////////////////////////////////////
  // Peripheral clocks
  // ////////////////////////////////////////////////////////////////////

  // USART1 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_USART1SEL) | RCC_DCKCFGR2_USART1SEL_0);

  // USART2 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_USART2SEL) | RCC_DCKCFGR2_USART2SEL_0);

  // USART3 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_USART3SEL) | RCC_DCKCFGR2_USART3SEL_0);

  // UART4 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_UART4SEL) | RCC_DCKCFGR2_UART4SEL_0);

  // USART6 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_USART6SEL) | RCC_DCKCFGR2_USART6SEL_0);

  // UART7 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_UART7SEL) | RCC_DCKCFGR2_UART7SEL_0);

  // I2C1 (fed by SYSCLK)
  RCC->DCKCFGR2 =
      ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_I2C1SEL) | RCC_DCKCFGR2_I2C1SEL_0);

  // CLK48 (fed by PLL)
  RCC->DCKCFGR2 = ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_CK48MSEL) | 0);

  // SDMMC1 (fed by CLK48)
  RCC->DCKCFGR2 = ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_SDMMC1SEL) | 0);
}

extern "C" void Reset_Handler()
{
  __disable_irq();


  // Copy data from Flash to SRAM, assumes 4 byte alignment of DATA must be
  // correct in the link file.
  data_init();

  // Clear the BSS segment, assumes 4 byte alignment of BSS must be correct in
  // the link file.
  bss_init();

  // Enable FPU (Enable CP10 and CP11 coprocessors)
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2));

  // ////////////////////////////////////////////////////////////////////
  // Reset the RCC clock configuration to the default reset state
  // ////////////////////////////////////////////////////////////////////

  // Set HSION bit
  RCC->CR |= 0x00000001;

  // Reset CFGR register
  RCC->CFGR = 0x00000000;

  // Reset HSEON, CSSON and PLLON bits
  RCC->CR &= 0xFEF6FFFF;

  // Reset PLLCFGR register
  RCC->PLLCFGR = 0x24003010;

  // Reset HSEBYP bit
  RCC->CR &= 0xFFFBFFFF;

  // Disable all interrupts
  RCC->CIR = 0x00000000;

  // Configure the Vector Table location add offset address
  SCB->VTOR = FLASH_BASE;

  __ISB();
  __DSB();

  // Run ctors
  constructor_init();

  __ISB();
  __DSB();

  // Setup clocks
  InitClocks();
  InitCachesAndAccelerators();

  __ISB();
  __DSB();

  // Set the stack pointer as we will never return here
  extern uint32_t __stack;
  __set_MSP((uint32_t)&__stack);

  int main(void);

  main();
}
