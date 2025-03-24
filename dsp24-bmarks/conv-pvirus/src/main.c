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
/* Power virus designed to stress test the convolution accelerator and achieve maximum power consumption
   Expects a payload of 1 64-bit uint specifying how many milliseconds to run the power virus for.
   Returns no payload
*/

#include "main.h"
#include "chip_config.h"

void mac_pv_intrinsics(uint64_t mt_cycles) {
  int32_t op1[32];

  srand(0xdeadbeef);
  for (int i = 0; i < 16; i++) {
    op1[i] = rand();
  }

  uint64_t start_time = clint_get_time(CLINT);
  uint64_t target_cycles = start_time + mt_cycles;

  uint16_t in_kernel[8] = {0x4000, 0xDEAD, 0xBEEF, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA};

  start_roi();
  while(clint_get_time(CLINT) < target_cycles) {
    set_conv_params(16, 1, ((uint64_t*) in_kernel));
    write_conv_dma(0, 16, op1);
    start_conv();
    read_conv_dma(0, 16, CONV_BASE + 0x20);
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
    uint64_t cycles = (*((uint64_t*) &t.payload)) * chip_mtime_freq / 500;

    switch (t.testid) {
      case 0:
        mac_pv_intrinsics(cycles);
        break;
      // case 1:
      //   mac_pv_asm(cycles);
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