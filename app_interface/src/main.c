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
#define DMA_ADDR1 0x87000000L

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

    printf("\n[STARTING TEST]\n\n");

    reset_fft();
    enable_Crack();

    write_fft_dma(1, 128, fft_data);
    uint64_t start_time = READ_CSR("mcycle");
    uint64_t start_instructions = READ_CSR("minstret");

    while(fft_busy() || fft_count_left()){
      continue;
        printf("pain:%d, %d \n", fft_busy(), fft_count_left());
    }; // This is needed since fft is blocking and is not a very good block

    uint64_t end_time = READ_CSR("mcycle");
    uint64_t end_instructions = READ_CSR("minstret");

    read_fft_real_dma(1, 128, DMA_ADDR1);
    printf("[DONE] Waiting Write\n");
    printf("mcycle = %lu\r\n", end_time - start_time);
    printf("minstret = %lu\r\n", end_instructions - start_instructions);
    uint32_t poll, real, imag;
    // for(int i=0; i<512; i++) {
    //     poll = reg_read32(DMA_ADDR1 + i*8);
    //     real = poll & 0xFFFF; 
    //     imag = (poll >> 16);
    //     printf("[%d]real: (%hd), imag: (%hd)\n", i, real, imag);
    // }
    for(int i=0; i<256; i++) {
        poll = reg_read16(DMA_ADDR1 + i*4);
        printf("[%d]real: (%hd)\n", i, poll);
    }
    
    printf("[DONE] Test\n");

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