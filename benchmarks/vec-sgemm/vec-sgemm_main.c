// See LICENSE for license details.

//**************************************************************************
// SGEMM benchmark
//--------------------------------------------------------------------------
//
// This benchmark tests a vectorized sgemm implementation.

#include <string.h>
#include "util.h"

//--------------------------------------------------------------------------
// Input/Reference Data

#include "dataset1.h"
#include "riscv.h"
#include "chip_config.h"
#include "encoding.h"

//--------------------------------------------------------------------------
// Main
extern volatile uint64_t tohost;
extern volatile uint64_t fromhost;

void *vec_sgemm_nn (size_t, size_t, size_t, const float*, size_t, const float*, size_t, float*, size_t);

void* memset(void* dest, int byte, size_t len)
{
  if ((((uintptr_t)dest | len) & (sizeof(uintptr_t)-1)) == 0) {
    uintptr_t word = byte & 0xFF;
    word |= word << 8;
    word |= word << 16;
    word |= word << 16 << 16;

    uintptr_t *d = dest;
    while (d < (uintptr_t*)(dest + len))
      *d++ = word;
  } else {
    char *d = dest;
    while (d < (char*)(dest + len))
      *d++ = byte;
  }
  return dest;
}

void app_init() {
  // torch::executor::runtime_init();
}

void vec_sgemm_main()
{
  int results_data[ARRAY_SIZE] = {0};

#if PREALLOCATE
  // If needed we preallocate everything in the caches
  vec_sgemm_nn(DIM_SIZE, DIM_SIZE, DIM_SIZE, input1_data, DIM_SIZE, input2_data, DIM_SIZE, results_data, DIM_SIZE);
  memset(results_data, 0, sizeof(results_data));
#endif

  // Do the sgemm
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");
  vec_sgemm_nn(DIM_SIZE, DIM_SIZE, DIM_SIZE, input1_data, DIM_SIZE, input2_data, DIM_SIZE, results_data, DIM_SIZE);
  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  printf("mcycle = %lu\r\n", end_time - start_time);
  printf("minstret = %lu\r\n", end_instructions - start_instructions);

  // Check the results
  return verifyFloat( ARRAY_SIZE, results_data, verify_data );
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Configure the system clock */
  /* Configure the system clock */
  
  /* USER CODE BEGIN SysInit */
  UART_InitType UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART_init_config);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */  
  /* USER CODE BEGIN Init */
  app_init();
  /* USER CODE END Init */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    vec_sgemm_main();
    return 0;
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