#include "hal_rcc.h"

void set_all_clocks(ClockSel_Type* clksel, uint32_t clksrc) {
  clksel->UNCORE = clksrc;
  clksel->TILE0  = clksrc;
  clksel->TILE1  = clksrc;
  clksel->TILE2  = clksrc;
  clksel->TILE3  = clksrc;
  clksel->CLKTAP = clksrc;
}