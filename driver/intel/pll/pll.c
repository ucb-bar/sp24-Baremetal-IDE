
#include "pll.h"

void configure_pll(PLL_Type* pll, uint32_t ratio, uint32_t fraction) {
  pll->PLLEN = 0;
  pll->MDIV_RATIO = 1;
  pll->RATIO = ratio;
  pll->FRACTION = fraction;
  pll->ZDIV0_RATIO = 1;
  pll->ZDIV1_RATIO = 1;
  pll->LDO_ENABLE = 1;
  pll->PLLEN = 1;
  pll->POWERGOOD_VNN = 1;
  pll->PLLFWEN_B = 1;
}