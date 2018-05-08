#include <crect/crect.hpp>

int main()
{
  /* Initialization code */
  crect::initialize();

  /*
   * Convoluted way to blink a LED
   */
  // crect::pend<J1>();

  while(1)
  {
    for (volatile unsigned i = 0; i < 0xFFFFFFFF; i++)
      ;
    // asm volatile("nop");
    // __WFI();
  }
}
