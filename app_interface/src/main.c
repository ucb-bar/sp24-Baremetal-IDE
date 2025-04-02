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
#include "chip_config.h"
#include "dataset2.h"
#include "../goldenmodel/fft_data_128len_131c.h"
#include "../goldenmodel/fft_data_128len_twinkle.h"
#include "../goldenmodel/fft_expected_data_128len_131c.h"
#include "../goldenmodel/fft_expected_data_128len_twinkle.h"
#define DMA_ADDR1 0x87000000L // Base Address
#define INPUT_ADDR1 0x08000000U // Where to save data - scratchpad is 0x08000000U
#define INPUT_DATA fft_data_131c
#define OUTPUT_DATA fft_expected_data_131c
#define NUM_POINTS 128
#define DMA_NUM 1

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN PUC */

void app_init() {
  // torch::executor::runtime_init();
}

void app_main() {
  uint64_t mhartid = READ_CSR("mhartid");

  printf("\r\n[STARTING TEST]\r\n");

  reset_fft();
  // enable_Crack(); // bad idea to enable for initial tests 

  write_fft_dma(DMA_NUM, NUM_POINTS, INPUT_DATA); 
  // sim test was like:
  // write_fft_dma(0, NUM_POINTS, (uint32_t*)fft_data[i]);
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");

  while(fft_busy() || fft_count_left()){
    // continue; // not sure why this was added
    printf("pain:%d, %d \r\n", fft_busy(), fft_count_left());
  }; // This is needed since fft is blocking and is not a very good block

  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  /* Making use of DMA */

  // read_fft_real_dma(1, NUM_POINTS, DMA_ADDR1);

  /* Not making use of DMA */

  uint32_t poll;
  for(int j=0; j<NUM_POINTS; j++) {
    poll = read_fft();
    int16_t poll_real = (int16_t) poll;
    int16_t expected_real = (int16_t) OUTPUT_DATA[0][j];
    // if (poll_real - expected_real < -MAX_DIFF || poll_real - expected_real > MAX_DIFF) {
    //   printf("[FAIL, test=%d, idx=%d] Expected %lx, received %lx]\n", i, j, fft_expected_data[i][j], poll);
    //   // error_cnt++;
    // }
    printf("Actual: %d, Expected: %d \r\n", poll_real, expected_real);
  }

  // printf("[DONE] Waiting Write\r\n");
  printf("mcycle = %lu\r\n", end_time - start_time);
  printf("minstret = %lu\r\n", end_instructions - start_instructions);
  // uint32_t poll, real, imag;
  // // for(int i=0; i<512; i++) {
  // //     poll = reg_read32(DMA_ADDR1 + i*8);
  // //     real = poll & 0xFFFF; 
  // //     imag = (poll >> 16);
  // //     printf("[%d]real: (%hd), imag: (%hd)\r\n", i, real, imag);
  // // }
  // for(int i=0; i<256; i++) {
  //     poll = reg_read16(DMA_ADDR1 + i*4);
  //     printf("[%d]real: (%hd)\r\n", i, poll);
  // }
  
  printf("[DONE] Test\r\n");

}
/* USER CODE END PUC */

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
    app_main();
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