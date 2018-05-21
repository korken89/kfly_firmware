//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "stm32f7xx.h"
#include "board/kflydc_v1.hpp"
#include "drivers/gpio/gpio.hpp"

namespace kfly_firmware
{
void board::init()
{
  reset_clocks();
  init_accelerators();
  init_clocks();
  init_peripherals();
}

void board::reset_clocks()
{
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
}

void board::init_clocks()
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

  // CLK48MUX (fed by PLLQ)
  RCC->DCKCFGR2 = ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_CK48MSEL) | 0);

  // SDMMC1 (fed by CLK48)
  RCC->DCKCFGR2 = ((RCC->DCKCFGR2 & ~RCC_DCKCFGR2_SDMMC1SEL) | 0);
}

void board::init_accelerators()
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

void board::init_peripherals()
{
  // ////////////////////////////////////////////////////////////////////
  // Enable peripherals
  // ////////////////////////////////////////////////////////////////////

  // AHB1 (GPIOs, DMA1, DMA2)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
                  RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

  // AHB2 (USB, RNG)
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN | RCC_AHB2ENR_RNGEN;

  // APB1 (USART2, USART3, UART4, UART7, I2C1, SPI3, TIM3, TIM4, TIM5)
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN |
                  RCC_APB1ENR_UART4EN | RCC_APB1ENR_UART7EN |
                  RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI3EN | RCC_APB1ENR_TIM3EN |
                  RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN;

  // APB2 (TIM1, TIM9, SPI1, ADC1, USART1, USART6, SDMMC1)
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_SPI1EN |
                  RCC_APB2ENR_ADC1EN | RCC_APB2ENR_USART1EN |
                  RCC_APB2ENR_USART6EN | RCC_APB2ENR_SDMMC1EN;

  // Configure system peripherals
  init_gpios();
  init_sensor_communication();
}

void board::init_gpios()
{
  using namespace gpio;

  // Setup all pins to their correct function
  gpio::config_all_banks<
      // Board LEDs
      gpio::pin_config< gpio::port::b, 12, gpio::mode::output,
                        gpio::output_type::open_drain,
                        gpio::pull_up_down::none >,

      gpio::pin_config< gpio::port::b, 13, gpio::mode::output,
                        gpio::output_type::open_drain,
                        gpio::pull_up_down::none >,

      // Safety button LED
      gpio::pin_config< gpio::port::e, 4, gpio::mode::output,
                        gpio::output_type::open_drain,
                        gpio::pull_up_down::none >,

      // IMU Heater
      gpio::pin_config< gpio::port::e, 12, gpio::mode::output >,

      // Sensor Voltage rail on/off
      gpio::pin_config< gpio::port::e, 15, gpio::mode::output >

      // END
      >();

  // Default of pins
  gpio::write< gpio::port::b, 12, gpio::state::high >();  // LED off
  gpio::write< gpio::port::b, 13, gpio::state::high >();  // LED off
  gpio::write< gpio::port::e, 4, gpio::state::high >();   // Safety LED off
  gpio::write< gpio::port::e, 12, gpio::state::low >();   // Heater off
  gpio::write< gpio::port::e, 15, gpio::state::low >();   // Sensor power off
}

void board::init_sensor_communication()
{
  // GPIOs

  // SPI

  // Timers

  // DMA
}
}  // END namespace kfly_firmware