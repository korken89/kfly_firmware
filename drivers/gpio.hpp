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

/// \brief Holdning structure for a GPIO's mask.
struct gpio_mask
{
  uint32_t MODER = 0;
  uint32_t OTYPER = 0;
  uint32_t OSPEEDR = 0;
  uint32_t PUPDR = 0;
  uint32_t AFR0 = 0;
  uint32_t AFR1 = 0;
};

/// \brief Class for generating GPIO masks from `pin_config`s.
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
    ((masks[static_cast< uint32_t >(Configs::Port_)].MODER |=
      (static_cast< uint32_t >(Configs::Mode) << (2 * Configs::Pin_))),
     ...);
    ((masks[static_cast< uint32_t >(Configs::Port_)].OTYPER |=
      (static_cast< uint32_t >(Configs::Output_type) << Configs::Pin_)),
     ...);
    ((masks[static_cast< uint32_t >(Configs::Port_)].OSPEEDR |=
      (static_cast< uint32_t >(Configs::Output_speed) << (2 * Configs::Pin_))),
     ...);
    ((masks[static_cast< uint32_t >(Configs::Port_)].PUPDR |=
      (static_cast< uint32_t >(Configs::Pull_up_down) << (2 * Configs::Pin_))),
     ...);
    ((masks[static_cast< uint32_t >(Configs::Port_)].AFR0 |=
      afr0_helper< Configs::Alternate_function, Configs::Pin_ >()),
     ...);
    ((masks[static_cast< uint32_t >(Configs::Port_)].AFR1 |=
      afr1_helper< Configs::Alternate_function, Configs::Pin_ >()),
     ...);
  }
};

/// \brief Helper function for writing a mask to the hardware.
/// \tparam P   	The port enum.
/// \param mask   The mask to be written to hardware.
template < port P >
inline void set_bank(const gpio_mask& mask)
{
  auto io = details::port_to_GPIO< P >();

  io->MODER = mask.MODER;
  io->OTYPER = mask.OTYPER;
  io->OSPEEDR = mask.OSPEEDR;
  io->PUPDR = mask.PUPDR;
  io->AFR[0] = mask.AFR0;
  io->AFR[1] = mask.AFR1;
}
}  // END namespace details

/// \brief Configuration of a GPIO pin.
/// \tparam Port  The port in which the pin to configure is located.
/// \tparam Pin   The pin to configure.
/// \tparam M     Pin mode setting.
/// \tparam OT    Output type setting.
/// \tparam PUD   Pull up/down setting.
/// \tparam OS    Output speed setting.
/// \tparam AF    Alternate function setting, is only used if mode is set to AF.
template <
    port Port, unsigned Pin, mode M, output_type OT = output_type::push_pull,
    pull_up_down PUD = pull_up_down::down, output_speed OS = output_speed::low,
    alternate_function AF = alternate_function::af0 >
struct pin_config
{
  static constexpr port Port_ = Port;
  static constexpr unsigned Pin_ = Pin;
  static constexpr mode Mode = M;
  static constexpr output_type Output_type = OT;
  static constexpr pull_up_down Pull_up_down = PUD;
  static constexpr output_speed Output_speed = OS;
  static constexpr alternate_function Alternate_function = AF;
};

/// \brief Traits to check for configs.
template < typename >
struct is_pin_config : std::false_type
{
};

/// \brief Traits to check for configs.
template < port Port, unsigned Pin, mode M, output_type OT, pull_up_down PUD,
           output_speed OS, alternate_function AF >
struct is_pin_config< pin_config< Port, Pin, M, OT, PUD, OS, AF > >
    : std::true_type
{
};

/// \brief Configures a GPIO pin, without altering other pins.
/// \tparam P     The port in which the pin to configure is located.
/// \tparam Pin   The pin to configure.
/// \tparam M     Pin mode setting.
/// \tparam OT    Output type setting.
/// \tparam PUD   Pull up/down setting.
/// \tparam OS    Output speed setting.
/// \tparam AF    Alternate function setting, is only used if mode is set to AF.
template < typename Config >
inline void config_pin()
{
  static_assert(is_pin_config< Config >(), "Is not a config.");
  static_assert(Config::Pin_ < 16, "Invalid pin number.");

  auto io = details::port_to_GPIO< Config::Port_ >();

  //
  // TODO: Add clearing of bits first
  //
  if constexpr (Config::Mode != mode::input)
    io->MODER |= (static_cast< unsigned >(Config::Mode) << (2 * Config::Pin_));

  if constexpr (Config::Output_type != output_type::push_pull)
    io->OTYPER |=
        (static_cast< unsigned >(Config::Output_type) << Config::Pin_);

  if constexpr (Config::Output_speed != output_speed::low)
    io->OSPEEDR |=
        (static_cast< unsigned >(Config::Output_speed) << (2 * Config::Pin_));

  if constexpr (Config::Pull_up_down != pull_up_down::none)
    io->PUPDR |=
        (static_cast< unsigned >(Config::Pull_up_down) << (2 * Config::Pin_));

  if constexpr (Config::Mode == mode::alternate_function)
  {
    if constexpr (Config::Pin < 8)
      io->AFR[0] |= (static_cast< unsigned >(Config::Alternate_function)
                     << (4 * Config::Pin_));
    else
      io->AFR[1] |= (static_cast< unsigned >(Config::Alternate_function)
                     << (4 * (Config::Pin_ - 8)));
  }
}

/// \brief Configures all GPIO pins, overwriting any existing config.
/// \tparam Configs   Variadic list of configurations (pin_configs).
template < typename... Configs >
void config_all_banks()
{
  static_assert((... && is_pin_config< Configs >()),
                "Config error, at least one input is not a config!");

  // Generate config masks
  constexpr details::gpio_masks generated_masks(Configs{}...);

  // Write configs to registers
  details::set_bank< port::a >(
      generated_masks.masks[static_cast< int >(port::a)]);
  details::set_bank< port::b >(
      generated_masks.masks[static_cast< int >(port::b)]);
  details::set_bank< port::c >(
      generated_masks.masks[static_cast< int >(port::c)]);
  details::set_bank< port::d >(
      generated_masks.masks[static_cast< int >(port::d)]);
  details::set_bank< port::e >(
      generated_masks.masks[static_cast< int >(port::e)]);
  details::set_bank< port::f >(
      generated_masks.masks[static_cast< int >(port::f)]);
  details::set_bank< port::g >(
      generated_masks.masks[static_cast< int >(port::g)]);
  details::set_bank< port::h >(
      generated_masks.masks[static_cast< int >(port::h)]);
  details::set_bank< port::i >(
      generated_masks.masks[static_cast< int >(port::i)]);
  details::set_bank< port::j >(
      generated_masks.masks[static_cast< int >(port::j)]);
  details::set_bank< port::k >(
      generated_masks.masks[static_cast< int >(port::k)]);
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

}  // END namespace gpio
