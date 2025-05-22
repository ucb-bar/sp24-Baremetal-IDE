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


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "chip_config.h"
#include "bmi088.h"
#include <math.h>



void app_init() {
  // +------------------------------------------------+
  // | Startup
  // +------------------------------------------------+
  
}

void app_main() {
  uint64_t mhartid = READ_CSR("mhartid");

  while (1) {
    printf("Hello from hart : %d\r\n", mhartid);
    msleep(100);

  }
}

void setup_pll() {
  printf("Finished setting up PLL\r\n");
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(int argc, char **argv) {
  /* MCU Configuration--------------------------------------------------------*/
  // Initialize UART0 for Serial Monitor
  UART_InitType UART0_init_config;
  UART0_init_config.baudrate = 115200;
  UART0_init_config.mode = UART_MODE_TX_RX;
  UART0_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART0_init_config);


  // /* Initialize the PLL so that we can run at 500MHz */
  // setup_pll();
  sleep(2);


  /* Initialize all configured peripherals */
  printf("-----Initialize App-----\r\n");
  app_init();

  /* Infinite loop */
  printf("-----Start Main-----\r\n");
  while (1) {
    app_main();
  }
  return 0;
}

/*
 * Main function for secondary harts
 *
 * Multi-threaded programs should provide their own implementation.
 */
void __attribute__((weak, noreturn)) __main(void) {
  uint64_t mhartid = READ_CSR("mhartid");
  while (1) {
    asm volatile("wfi");
  }
}