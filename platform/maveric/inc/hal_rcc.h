/**
 * @file hal_rcc.h
 * @author Jasmine Angle | angle@berkeley.edu
 * @brief
 * @version 0.1
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __HAL_RCC_H__
#define __HAL_RCC_H__

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

void set_all_clocks(ClockSel_Type* clksel, uint32_t clksrc);

#ifdef __cplusplus
}
#endif

#endif // __HAL_RCC_H__
