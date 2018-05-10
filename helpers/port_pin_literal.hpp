//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <cstdint>
#include <type_traits>
#include "helpers/constant.hpp"
#include "drivers/gpio_defs.hpp"

#pragma once


namespace details
{
/// \brief Check if str only has numbers in it.
template < unsigned N >
constexpr bool is_number(const char (&str)[N], unsigned start = 0,
                         unsigned end = N)
{
  for (unsigned i = start; i < end; i++)
    if (str[i] > '9' || str[i] < '0')
      return false;

  return (start != end);
}

/// \brief string to number within [start, end).
template < unsigned N >
constexpr unsigned to_number(const char (&str)[N], unsigned start = 0,
                             unsigned end = N)
{
  unsigned num = 0;

  for (unsigned i = start; i < end; i++)
    num = num * 10 + (str[i] - '0');

  return num;
}

/// \brief Checks pin/port definition: <PORT>.<PIN>
///        Example: 1.22, 0.31, 0.0, 5.1
template < unsigned N >
constexpr bool is_pin_port(const char (&str)[N], unsigned dot_pos)
{
  return is_number(str, 0, dot_pos) && is_number(str, dot_pos + 1, N);
}

/// \brief Finds the dot in the string.
template < unsigned N >
constexpr unsigned find_dot(const char (&str)[N])
{
  unsigned pos = 0;
  for (char ch : str)
  {
    if (ch == '.')
      return pos;

    pos++;
  }

  return N;
}
}  // END namespace details

/// \brief Abstraction for holding port & pin definitions.
/// \tparam Port  Port number
/// \tparam Pin   Pin number
template < unsigned Port, unsigned Pin >
struct pin_location
{
  using port = constant< gpio::to_port< Port > >;
  using pin = constant< Pin >;
};

/// \brief User defined literal for <PORT>.<PIN>_pin
template < char... Str >
constexpr auto operator"" _pin()
{
  constexpr const char str[] = {Str...};
  constexpr auto N = sizeof...(Str);

  // Get dot position
  constexpr auto dot_pos = find_dot(str);

  // Check format
  static_assert(is_pin_port(str, dot_pos),
                "Pin error! Format <PORT>.<PIN>_pin");

  // Make pin_location object
  return pin_location< to_number(str, 0, dot_pos),
                       to_number(str, dot_pos + 1, N) >{};
}
