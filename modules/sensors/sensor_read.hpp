//          Copyright Emil Fresk 2017-2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

namespace kfly_firmware
{
//
// High level IMU class
class imu
{
  // Actual sensor data
public:
  // Initialization and setup
  void init_sensor_read();

  // Starts IMU sensor read
  void imu_start_read();

  // Starts barobeter sensor read
  void barometer_start_read();
};
}
