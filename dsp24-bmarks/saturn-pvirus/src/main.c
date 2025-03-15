/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* Power virus designed to stress test the saturn core and achieve maximum power consumption
   Expects a payload of 1 64-bit uint specifying how many cycles to run the power virus for, automatically rounded down to a multiple
   of 1000. Returns no payload
*/

#include "main.h"
#include <riscv_vector.h>

void mac_pv(uint64_t cycles) {
  int32_t op1[32];
  int32_t op2[32];

  srand(0xdeadbeef);
  for (int i = 0; i < 32; i++) {
    op1[i] = rand();
    op2[i] = rand();
  }

  size_t vl = __riscv_vsetvlmax_e8m4();
  vint8m4_t fac1 = __riscv_vle8_v_i8m4((int8_t*) &op1, vl);
  vint8m4_t fac2 = __riscv_vle8_v_i8m4((int8_t*) &op2, vl);

  uint64_t start_time = clint_get_time(CLINT);
  uint64_t target_cycles = start_time + (cycles/1000);

  start_roi();
  while(clint_get_time(CLINT) < target_cycles) {
    vint8m4_t acc = __riscv_vmacc_vv_i8m4(acc, fac1, fac2, vl);
  }
  end_roi();
  xmit_payload_packet(NULL, 0);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  while (1) {
    test_info t = init_test(UART1);
    uint64_t cycles = *((uint64_t*) &t.payload);
    switch (t.testid) {
      case 0:
        mac_pv(cycles);
    }
  }


  /* USER CODE END WHILE */
}

/*
 * Main function for secondary harts
 * 
 * Multi-threaded programs should provide their own implementation.
 */
void __attribute__((weak, noreturn)) __main(void) {
  while (1) {
   asm volatile ("wfi");
  }
}