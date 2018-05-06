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
    //__WFI();
  }
}
