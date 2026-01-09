/**
 * @file gpio.c
 * @author -T.K.- / t_k_233@outlook.com
 * @brief 
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "gpio.h"

void gpio_init(GPIO_Type *GPIOx, GPIO_InitType *GPIO_init, GPIO_Pin pin) {
  if (GPIO_init->mode == GPIO_MODE_INPUT) {
    CLEAR_BITS(GPIOx->IOF_EN, (uint32_t)pin);
    
    SET_BITS(GPIOx->INPUT_EN, (uint32_t)pin);
    CLEAR_BITS(GPIOx->OUTPUT_EN, (uint32_t)pin);
  }
  else if (GPIO_init->mode == GPIO_MODE_OUTPUT) {
    CLEAR_BITS(GPIOx->IOF_EN, (uint32_t)pin);
    
    CLEAR_BITS(GPIOx->INPUT_EN, (uint32_t)pin);
    SET_BITS(GPIOx->OUTPUT_EN, (uint32_t)pin);
  }
  else if (GPIO_init->mode == GPIO_MODE_INOUT) {
    CLEAR_BITS(GPIOx->IOF_EN, (uint32_t)pin);

    SET_BITS(GPIOx->INPUT_EN, (uint32_t)pin);
    SET_BITS(GPIOx->OUTPUT_EN, (uint32_t)pin);
  }
  else if (GPIO_init->mode == GPIO_MODE_ALTERNATE_FUNCTION_0) {
    SET_BITS(GPIOx->IOF_EN, (uint32_t)pin);
    CLEAR_BITS(GPIOx->IOF_SEL, (uint32_t)pin);
  }
  else if (GPIO_init->mode == GPIO_MODE_ALTERNATE_FUNCTION_1) {
    SET_BITS(GPIOx->IOF_EN, (uint32_t)pin);
    SET_BITS(GPIOx->IOF_SEL, (uint32_t)pin);
  }

  if (GPIO_init->pull == GPIO_PULL_UP) {
    SET_BITS(GPIOx->PUE, (uint32_t)pin);
  }
  else {
    CLEAR_BITS(GPIOx->PUE, (uint32_t)pin);
  }

  if (GPIO_init->drive_strength == GPIO_DS_STRONG) {
    SET_BITS(GPIOx->DS, (uint32_t)pin);
  }
  else {
    CLEAR_BITS(GPIOx->DS, (uint32_t)pin);
  }  
}

uint8_t gpio_read_pin(GPIO_Type *GPIOx, GPIO_Pin pin) {
  return READ_BITS(GPIOx->INPUT_VAL, (uint32_t)pin) ? 1 : 0;
}

void gpio_invert_pin(GPIO_Type *GPIOx, GPIO_Pin pin) {
  if (READ_BITS(GPIOx->OUT_XOR, (uint32_t)pin)) {
    CLEAR_BITS(GPIOx->OUT_XOR, (uint32_t)pin);
  } else {
    SET_BITS(GPIOx->OUT_XOR, (uint32_t)pin);
  }
  return;
}

void gpio_write_pin(GPIO_Type *GPIOx, GPIO_Pin pin, uint8_t value) {
  if (value) {
    SET_BITS(GPIOx->OUTPUT_VAL, (uint32_t)pin);
  }
  else {
    CLEAR_BITS(GPIOx->OUTPUT_VAL, (uint32_t)pin);
  }
  return;
}

void gpio_toggle_pin(GPIO_Type *GPIOx, GPIO_Pin pin) {
  if (READ_BITS(GPIOx->OUTPUT_VAL, (uint32_t)pin)) {
    CLEAR_BITS(GPIOx->OUTPUT_VAL, (uint32_t)pin);
  } else {
    SET_BITS(GPIOx->OUTPUT_VAL, (uint32_t)pin);
  }
  return;
}

void gpio_enable_rise_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  SET_BITS(GPIOx->RISE_IE, (uint32_t)pin);
}

void gpio_disable_rise_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  CLEAR_BITS(GPIOx->RISE_IE, (uint32_t)pin);
}

void gpio_enable_fall_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  SET_BITS(GPIOx->FALL_IE, (uint32_t)pin);
}

void gpio_disable_fall_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  CLEAR_BITS(GPIOx->FALL_IE, (uint32_t)pin);
}

void gpio_enable_high_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  SET_BITS(GPIOx->HIGH_IE, (uint32_t)pin);
}

void gpio_disable_high_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  CLEAR_BITS(GPIOx->HIGH_IE, (uint32_t)pin);
}

void gpio_enable_low_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  SET_BITS(GPIOx->LOW_IE, (uint32_t)pin);
}

void gpio_disable_low_interrupt(GPIO_Type *GPIOx, GPIO_Pin pin) {
  CLEAR_BITS(GPIOx->LOW_IE, (uint32_t)pin);
}