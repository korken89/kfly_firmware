#include <crect/crect.hpp>

int main()
{
  // Initialization code (enables interrupts)
  crect::initialize();

  while(1)
  {
    for (volatile unsigned i = 0; i < 0xFFFFFFFF; i++)
      ;
    // asm volatile("nop");
    // __WFI();
  }
}
