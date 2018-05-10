//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "drivers/gpio_defs.hpp"
#include "stm32f7xx.h"

#pragma once

namespace gpio
{
namespace details
{
static constexpr uint32_t ports[] = {
    GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE, GPIOF_BASE,
    GPIOG_BASE, GPIOH_BASE, GPIOI_BASE, GPIOJ_BASE, GPIOK_BASE};

template < port P >
inline GPIO_TypeDef* port_to_GPIO()
{
  return reinterpret_cast< GPIO_TypeDef* >(ports[static_cast< int >(P)]);
}
}

template < port P, unsigned Pin, mode M, output_type OT, output_speed OS,
           pull_up_down PUD, alternate_function AF = alternate_function::af0 >
inline void config_pin()
{
  static_assert(Pin < 16, "Invalid pin number.");

  auto io = details::port_to_GPIO< P >();

  io->MODER = (static_cast< unsigned >(M) << (2 * Pin));
  io->OTYPER = (static_cast< unsigned >(OT) << Pin);
  io->OSPEEDR = (static_cast< unsigned >(OS) << (2 * Pin));
  io->PUPDR = (static_cast< unsigned >(PUD) << (2 * Pin));

  if constexpr (M == mode::alternate_function)
  {
    if constexpr (Pin < 8)
      io->AFR[0] = (static_cast< unsigned >(AF) << (4 * Pin));
    else
      io->AFR[1] = (static_cast< unsigned >(AF) << (4 * (Pin - 8)));
  }
}

template < port P, unsigned Pin, state S >
inline void set()
{
  static_assert(Pin < 16, "Invalid pin number.");

  if constexpr (S == state::high)
    details::port_to_GPIO< P >()->BSRR = (1 << Pin);
  else
    details::port_to_GPIO< P >()->BSRR = (1 << (Pin + 16));
}

template < port P, unsigned Pin >
inline void toggle()
{
  static_assert(Pin < 16, "Invalid pin number.");

  details::port_to_GPIO< P >()->ODR ^= (1 << Pin);
}

template < port P, unsigned Pin >
inline state get()
{
  static_assert(Pin < 16, "Invalid pin number.");

  if (details::port_to_GPIO< P >()->IDR & (1 << Pin))
    return state::high;
  else
    return state::low;
}
}
