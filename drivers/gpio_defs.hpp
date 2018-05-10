//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

namespace gpio
{
/// \brief Available ports definition.
enum class port
{
  a = 0,
  b = 1,
  c = 2,
  d = 3,
  e = 4,
  f = 5,
  g = 6,
  h = 7,
  i = 8,
  j = 9,
  k = 10
};

/// \brief Possible pin states.
enum class state
{
  low = 0,
  high = 1
};

/// \brief Possible pin modes.
enum class mode
{
  input = 0,
  output = 1,
  alternate_function = 2,
  analog = 3
};

/// \brief Possible pin output types.
enum class output_type
{
  push_pull = 0,
  open_drain = 1
};

/// \brief Possible pin output speeds.
enum class output_speed
{
  low = 0,
  medium = 1,
  high = 2,
  extreme = 3
};

/// \brief Possible pin pull up/down settings.
enum class pull_up_down
{
  none = 0,
  up = 1,
  down = 2
};

/// \brief Possible pin alternate functions.
enum class alternate_function
{
  af0 = 0,
  af1 = 1,
  af2 = 2,
  af3 = 3,
  af4 = 4,
  af5 = 5,
  af6 = 6,
  af7 = 7,
  af8 = 8,
  af9 = 9,
  af10 = 10,
  af11 = 11,
  af12 = 12,
  af13 = 13,
  af14 = 14,
  af15 = 15,
};
}
