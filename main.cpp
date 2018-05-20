#include <crect/crect.hpp>
#include "board/kflydc_v1.hpp"
#include "drivers/gpio.hpp"

int main()
{
  // System (board) initialization
  kfly_firmware::board::init();

  gpio::config_all_banks<
      gpio::pin_config< gpio::port::a, 0, gpio::mode::output >,
      gpio::pin_config< gpio::port::a, 1, gpio::mode::output > >();

  // gpio::config_pin<
  // gpio::pin_config< gpio::port::a, 0, gpio::mode::output >> ();

  // Initialization code (enables interrupts)
  crect::initialize();

  while (1)
  {
    for (volatile unsigned i = 0; i < 0xFFFFFFFF; i++)
      ;
    // asm volatile("nop");
    // __WFI();
  }
}
