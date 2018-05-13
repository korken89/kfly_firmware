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

  __ISB();
  __DSB();

  // Run ctors
  constructor_init();

  __ISB();
  __DSB();

  // Set the stack pointer as we will never return here
  extern uint32_t __stack;
  __set_MSP((uint32_t)&__stack);

  int main(void);

  main();
}
