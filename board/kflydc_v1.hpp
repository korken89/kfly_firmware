//          Copyright Emil Fresk 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

namespace kfly_firmware
{
class board
{
private:
  static void reset_clocks();
  static void init_clocks();
  static void init_accelerators();
  static void init_peripherals();
  static void init_sensor_communication();

public:
  static void init();
};
}  // END namespace kfly_firmware
