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
#include "main.h"

typedef struct __attribute__((packed)) {
  uint32_t time_ms;
} HostPayload;


void idle_test(uint64_t cycles) {
    start_roi();
    uint64_t target_tick = get_cycles() + cycles;
    while (get_cycles() < target_tick) {
      asm volatile("nop");
    }

    end_roi();
    char* payload = "Hello World!";
    sleep(2);
    xmit_payload_packet(payload, strlen(payload));

}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  while (1) {
    volatile test_info t = init_test(UART0);
    uint64_t cycles = ((HostPayload*) &t.payload)->time_ms * chip_freq / 1000;
    switch (t.testid) {
      default:
        idle_test(cycles);
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