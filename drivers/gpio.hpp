//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "stm32f7xx.h"
#include "drivers/gpio_defs.hpp"
#include "helpers/port_pin_literal.hpp"
#include "esl/esl.hpp"

#pragma once

namespace gpio
{
namespace details
{
/// \brief Converts a gpio::port enum into a pointer to the correct GPIO.
/// \tparam P   The port enum.
/// \return     Pointer to GPIO port.
template < port P >
inline GPIO_TypeDef* port_to_GPIO()
{
  static constexpr uint32_t ports[] = {
      GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE, GPIOF_BASE,
      GPIOG_BASE, GPIOH_BASE, GPIOI_BASE, GPIOJ_BASE, GPIOK_BASE};

  return reinterpret_cast< GPIO_TypeDef* >(ports[static_cast< int >(P)]);
}
}  // END namespace details

/// \brief Configures a GPIO pin, without altering other pins;
/// \tparam P     The port in which the pin to configure is located.
/// \tparam Pin   The pin to configure.
/// \tparam M     Pin mode setting.
/// \tparam OT    Output type setting.
/// \tparam PUD   Pull up/down setting.
/// \tparam OS    Output speed setting.
/// \tparam AF    Alternate function setting, is only used if mode is set to AF.
template < port P, unsigned Pin, mode M, output_type OT,
           pull_up_down PUD = pull_up_down::down,
           output_speed OS = output_speed::low,
           alternate_function AF = alternate_function::af0 >
inline void config_pin()
{
  static_assert(Pin < 16, "Invalid pin number.");

  auto io = details::port_to_GPIO< P >();

  if constexpr (M != mode::input)
    io->MODER |= (static_cast< unsigned >(M) << (2 * Pin));

  if constexpr (OT != output_type::push_pull)
    io->OTYPER |= (static_cast< unsigned >(OT) << Pin);

  if constexpr (OS != output_speed::low)
    io->OSPEEDR |= (static_cast< unsigned >(OS) << (2 * Pin));

  if constexpr (PUD != pull_up_down::none)
    io->PUPDR |= (static_cast< unsigned >(PUD) << (2 * Pin));

  if constexpr (M == mode::alternate_function)
  {
    if constexpr (Pin < 8)
      io->AFR[0] |= (static_cast< unsigned >(AF) << (4 * Pin));
    else
      io->AFR[1] |= (static_cast< unsigned >(AF) << (4 * (Pin - 8)));
  }
}

/// \brief Sets a GPIO pin high or low.
/// \tparam P     The port in which the pin to control is located.
/// \tparam Pin   The pin to control.
/// \tparam S     Pin state value (high/low).
template < port P, unsigned Pin, state S >
inline void write()
{
  static_assert(Pin < 16, "Invalid pin number.");

  if constexpr (S == state::high)
    details::port_to_GPIO< P >()->BSRR = (1 << Pin);
  else
    details::port_to_GPIO< P >()->BSRR = (1 << (Pin + 16));
}

/// \brief Toggles a GPIO pin.
/// \tparam P     The port in which the pin to control is located.
/// \tparam Pin   The pin to control.
template < port P, unsigned Pin >
inline void toggle()
{
  static_assert(Pin < 16, "Invalid pin number.");

  details::port_to_GPIO< P >()->ODR ^= (1 << Pin);
}

/// \brief Reads a GPIO pin.
/// \tparam P     The port in which the pin to read is located.
/// \tparam Pin   The pin to read.
/// \return       The state of the pin.
template < port P, unsigned Pin >
inline state read()
{
  static_assert(Pin < 16, "Invalid pin number.");

  if (details::port_to_GPIO< P >()->IDR & (1 << Pin))
    return state::high;
  else
    return state::low;
}

//
//
//
// Testing area
//
//
//
template < port P, unsigned Pin, mode M, output_type OT,
           pull_up_down PUD = pull_up_down::down,
           output_speed OS = output_speed::low,
           alternate_function AF = alternate_function::af0 >
struct gpio_config
{
  using port = constant< P >;
  using pin = constant< Pin >;
  using mode = constant< M >;
  using output_type = constant< OT >;
  using pull_up_down = constant< PUD >;
  using output_speed = constant< OS >;
  using alternate_function = constant< AF >;
};

struct gpio_mask
{
  uint32_t MODER = 0;
  uint32_t OTYPER = 0;
  uint32_t OSPEEDR = 0;
  uint32_t PUPDR = 0;
  uint32_t AFR0 = 0;
  uint32_t AFR1 = 0;
};

class gpio_masks
{
private:
  template < alternate_function AF, unsigned Pin >
  constexpr uint32_t afr0_helper()
  {
    if constexpr (Pin < 8)
      return static_cast< uint32_t >(AF) << (4 * Pin);
    else
      return 0;
  }

  template < alternate_function AF, unsigned Pin >
  constexpr uint32_t afr1_helper()
  {
    if constexpr (Pin < 8)
      return 0;
    else
      return static_cast< uint32_t >(AF) << (4 * (Pin - 8));
  }

public:
  gpio_mask masks[static_cast< uint32_t >(port::END_OF_PORTS)];

  template < typename... Configs >
  constexpr gpio_masks(Configs...)
  {
    ((masks[static_cast< uint32_t >(Configs::port)].MODER |=
      (static_cast< uint32_t >(Configs::M) << (2 * Configs::Pin))),
     ...);
    ((masks[static_cast< uint32_t >(Configs::port)].OTYPER |=
      (static_cast< uint32_t >(Configs::OT) << Configs::Pin)),
     ...);
    ((masks[static_cast< uint32_t >(Configs::port)].OSPEEDR |=
      (static_cast< uint32_t >(Configs::OS) << (2 * Configs::Pin))),
     ...);
    ((masks[static_cast< uint32_t >(Configs::port)].PUPDR |=
      (static_cast< uint32_t >(Configs::PUD) << (2 * Configs::Pin))),
     ...);
    ((masks[static_cast< uint32_t >(Configs::port)].AFR0 |=
      afr0_helper< Configs::AF, Configs::Pin >()),
     ...);
    ((masks[static_cast< uint32_t >(Configs::port)].AFR1 |=
      afr1_helper< Configs::AF, Configs::Pin >()),
     ...);
  }
};

template < typename... Configs >
inline void config_gpio(Configs... configs)
{
  // Generate config masks
  constexpr gpio_masks masks(configs...);

  // Write configs to registers
  esl::repeat< static_cast< std::size_t >(port::END_OF_PORTS) >([&](auto idx) {
    auto io = details::port_to_GPIO< static_cast< port >(idx) >();
    const auto &mask = masks.masks[idx];

    io->MODER = mask.MODER;
    io->OTYPER = mask.OTYPER;
    io->OSPEEDR = mask.OSPEEDR;
    io->PUPDR = mask.PUPDR;
    io->AFR[0] = mask.AFR0;
    io->AFR[1] = mask.AFR1;
  });
}

}  // END namespace gpio
