#ifndef __LL_PLL_H
#define __LL_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "metal.h"

typedef struct {
  __IO uint32_t UNCORE;                                 // 0x00
  __IO uint32_t TILE0;                                  // 0x04
  __IO uint32_t TILE1;                                  // 0x08
  __IO uint32_t TILE2;                                  // 0x0C
  __IO uint32_t TILE3;                                  // 0x10
  __IO uint32_t CLKTAP;                                 // 0x14
} ClockSel_Type;

#ifdef __cplusplus
}
#endif

#endif /* __LL_PLL_H */