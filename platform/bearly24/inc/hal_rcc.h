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

#include "ll_rcc.h"

void set_all_clocks(ClockSel_Type* clksel, uint32_t clksrc);

#ifdef __cplusplus
}
#endif

#endif // __HAL_RCC_H__
