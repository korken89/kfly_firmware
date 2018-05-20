#include <crect/crect.hpp>
#include "board/kflydc_v1.hpp"
#include "drivers/gpio.hpp"

int main()
{
  // System (board) initialization
  kfly_firmware::board::init();

  // Initialization code (enables interrupts)
  crect::initialize();

  while (1)
  {
    for (volatile unsigned i = 0; i < 200000000; i++)
      ;
    // asm volatile("nop");
    // __WFI();
  }
}
