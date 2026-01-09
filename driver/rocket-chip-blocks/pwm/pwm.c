/**
 * @file hal_pwm.c
 * @author -Ethan Gao / eygao@berkeley.edu
 * @brief
 * @version 0.1
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "pwm.h"

extern uint64_t sys_clk_freq;

uint32_t log2_bitwise(uint32_t n) { 
    uint32_t log = 0; 
 
    while (n >>= 1) {  // Right shift n until it becomes 0 
        log++; 
    } 
    return log;  // log will be the position of the highest set bit 
} 

void pwm_init(PWM_Type *PWMx, PWM_InitType *PWM_init) {
  PWMx->PWM_CFG = *((uint32_t*) PWM_init);
  PWMx->PWM_CMP0 = 0;
  PWMx->PWM_CMP1 = 0;
  PWMx->PWM_CMP2 = 0;
  PWMx->PWM_CMP3 = 0;
  printf("configured PWM\n");
}

void pwm_stop(PWM_Type *PWMx, uint32_t idx) {
  pwm_set_compare_value(PWMx, idx, 0);
}

void pwm_set_frequency(PWM_Type *PWMx, uint32_t idx, double freq) {
  // TODO: implementation
  // PWM frequency = System clock / 2^pwmscale
  uint16_t pwmscale = 15;
  if (READ_BITS(PWMx->PWM_CFG, PWM_PWMZEROCMP_MSK) != 0) {
    while ((sys_clk_freq / (1<<pwmscale)) / freq <= 65535)
    {
      if (((sys_clk_freq / (1<<pwmscale)) % (int) freq == 0)) {
        break;
      }
      pwmscale = pwmscale - 1;
    }
    pwm_set_scale(PWMx, pwmscale);
  
    uint16_t cmp0 = sys_clk_freq / ((1<<pwmscale) * freq);
    //printf("CMP0 %d", cmp0);
    pwm_set_compare_value(PWMx, 0, cmp0);
  } else {
    if (sys_clk_freq / 65535 < freq) {
      printf("Cannot generate this frequency without using Zero Compare Function\n");
    } else {
      uint16_t pwmscale = (int) (log2_bitwise( (int)sys_clk_freq / 65535 / freq));
      pwm_set_scale(PWMx, pwmscale);
    }
  }
}

uint32_t pwm_get_frequency(PWM_Type *PWMx, uint32_t idx) {
  // TODO: implementation
  uint16_t pwmscale = READ_BITS(PWMx->PWM_CFG, PWM_PWMSCALE_MSK);
  uint16_t zecmp = READ_BITS(PWMx->PWM_CFG, PWM_PWMZEROCMP_MSK);
  if (READ_BITS(PWMx->PWM_CFG, PWM_PWMZEROCMP_MSK) != 0) {
    return sys_clk_freq / ((1<<pwmscale)*(PWMx->PWM_CMP0));
  } else {
    return sys_clk_freq / (1<<pwmscale) / 65535;
  }
  // return 0;
}

void pwm_set_duty_cycle(PWM_Type *PWMx, uint32_t idx, uint32_t duty, int phase_corr) {
  // TODO: check edge cases 0 and 100% duty cycle for 100% cmp=0 and for 0% cmpX > cmp0 or > 65535
  uint16_t pwmscale = READ_BITS(PWMx->PWM_CFG, PWM_PWMSCALE_MSK);
  uint32_t cmpvalue = 0;
  uint32_t freq = pwm_get_frequency(PWMx, idx);
  uint16_t zecmp = READ_BITS(PWMx->PWM_CFG, PWM_PWMZEROCMP_MSK);
  duty = 100-duty;
  if (READ_BITS(PWMx->PWM_CFG, PWM_PWMZEROCMP_MSK) == 0){
    cmpvalue = ((double) duty/100) * (double) sys_clk_freq / ((1<<pwmscale) * freq); //The expression after duty cycle is equivalent to 65535 since that is the pwms max value
    //printf("CMP Value %d", cmpvalue);
    if (cmpvalue > 65535) {
      cmpvalue = 65535;
    }
    if (cmpvalue < 0) {
      cmpvalue = 0;
    }
    pwm_set_compare_value(PWMx, idx, cmpvalue);
  } else {
    if (idx == 0) {
      printf("You are using Zero Compare so changing the value of CMP0 will change the frequency of the PWM\n");
    } else {
      cmpvalue = ((double) duty/100) * PWMx->PWM_CMP0;
      //printf("CMP Value %d", cmpvalue);
      if (cmpvalue > 65535) {
      cmpvalue = 65535;
      }
      if (cmpvalue < 0) {
        cmpvalue = 0;
      }
      pwm_set_compare_value(PWMx, idx, cmpvalue);
    }
  }
}

uint32_t pwm_get_duty_cycle(PWM_Type *PWMx, uint32_t idx) {
  // TODO: implementation
  if (READ_BITS(PWMx->PWM_CFG, PWM_PWMZEROCMP_MSK) == 0){
    switch (idx) {
    case 0:
      return 100-100*((double)(PWMx->PWM_CMP0) / (double)(65535));
      break;
    case 1:
      return 100-100*((double)(PWMx->PWM_CMP1) / (double)(65535));
      break;
    case 2:
      return 100-100*((double)(PWMx->PWM_CMP2) / (double)(65535));
      break;
    case 3:
      return 100-100*((double)(PWMx->PWM_CMP3) / (double)(65535));
      break;
    }
  } else {
    switch (idx) {
    case 0:
      return 100-100*((double)(PWMx->PWM_CMP0) / (double)(PWMx->PWM_CMP0));
      break;
    case 1:
      return 100-100*((double)(PWMx->PWM_CMP1) / (double)(PWMx->PWM_CMP0));
      break;
    case 2:
      return 100-100*((double)(PWMx->PWM_CMP2) / (double)(PWMx->PWM_CMP0));
      break;
    case 3:
      return 100-100*((double)(PWMx->PWM_CMP3) / (double)(PWMx->PWM_CMP0));
      break;
    }
  }
  return 0; 
}

void pwm_trigger(PWM_Type *PWMx, uint32_t idx) {
  // TODO: implementation
}
