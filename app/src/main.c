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
#include "vec_conditional_dataset.h"
#include "chip_config.h"

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

uint8_t counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN PUC */

static int verify_short(int n, const volatile int16_t* test, const int16_t* verify)
{
  int i;
  // Unrolled for faster verification
  for (i = 0; i < n/2*2; i+=2)
  {
    int t0 = test[i], t1 = test[i+1];
    int v0 = verify[i], v1 = verify[i+1];
    if (t0 != v0) return i+1;
    if (t1 != v1) return i+2;
  }
  if (n % 2 != 0 && test[n-1] != verify[n-1])
    return n;
  return 0;
}
  

void vectorized_vec_conditional(int16_t* z, const int8_t* x, const int16_t* a, const int16_t* b, size_t n) {
  asm volatile (
      "1:                         \n"
      "vsetvli t0, %[n], e8, m1, ta, ma   \n"  
      "vle8.v v0, (%[x])          \n"  
      "sub %[n], %[n], t0         \n"  
      "add %[x], %[x], t0         \n"  
      "vmslt.vi v0, v0, 5         \n"  
      "vsetvli x0, x0, e16, m2, ta, mu    \n"  
      "slli t0, t0, 1             \n"  
      "vle16.v v2, (%[a]), v0.t   \n"  
      "vmnot.m v0, v0             \n"  
      "add %[a], %[a], t0         \n"  
      "vle16.v v2, (%[b]), v0.t   \n"  
      "add %[b], %[b], t0         \n"  
      "vse16.v v2, (%[z])         \n"  
      "add %[z], %[z], t0         \n"  
      "bnez %[n], 1b             \n"
      :
      : [z] "r"(z), [x] "r"(x), [a] "r"(a), [b] "r"(b), [n] "r"(n)
      : "t0", "x0", "v0", "v2", "memory"
  );
}

void naive_vec_conditional(size_t n, const int8_t* x, const int16_t* a, const int16_t* b, int16_t* z) {
    for (size_t i = 0; i < n; i++) {
        z[i] = (x[i] < 5) ? a[i] : b[i];
    }
}

void benchmark_vec_conditional() {
    int vector_len = 1000;
    uint8_t input[vector_len] = {};
    int16_t vectorized_output[vector_len];
    int16_t naive_output[vector_len];

    uint64_t vectorized_start_cycles = READ_CSR("mcycle");
    uint64_t vectorized_start_instructions = READ_CSR("minstret");

    // start prototyping

    // int8_t input_vector_1[10] = {0,   3,   1,   3,   1,   1,   8,   2,   9,   5};
    // int16_t input_vector_2[10] = {454, 564, 989, 350, 64, 584, 140,  6, 339, 392};
    // int16_t input_vector_3[10] = {833,   1, 749, 572, 949, 216, 621, 572, 890, 898};

    // int16_t vectorized_output_proto[10];
    // vectorized_vec_conditional(vectorized_output_proto, input_vector_1, input_vector_2, input_vector_3, 10);
    // end prototyping


    vectorized_vec_conditional(vectorized_output, vec_conditional_input1_data, vec_conditional_input2_data, vec_conditional_input3_data, vector_len);
    uint64_t vectorized_end_instructions = READ_CSR("minstret");
    uint64_t vectorized_end_cycles = READ_CSR("mcycle");
    
    uint64_t naive_start_cycles = READ_CSR("mcycle");
    uint64_t naive_start_instructions = READ_CSR("minstret");
    naive_vec_conditional(vector_len, vec_conditional_input1_data, vec_conditional_input2_data, vec_conditional_input3_data, naive_output);
    uint64_t naive_end_instructions = READ_CSR("minstret");
    uint64_t naive_end_cycles = READ_CSR("mcycle");

    if (verify_short(vector_len, vectorized_output, vec_conditional_verify_data) != 0) {
        printf("VECTORIZED output is not correct\n");
    }
    int differences = verify_short(vector_len, naive_output, vectorized_output);
    
    printf("VECTORIZED took %llu cycles and %llu instructions\n", vectorized_end_cycles - vectorized_start_cycles, vectorized_end_instructions - vectorized_start_instructions);
    printf("NAIVE took %llu cycles and %llu instructions\n", naive_end_cycles - naive_start_cycles, naive_end_instructions - naive_start_instructions);
    printf("naive was better by %llu cycles than vectorized on vec conditional\n", vectorized_end_cycles - naive_end_cycles);
}




void app_init() {
  // torch::executor::runtime_init();
}



void app_main() {
  uint64_t mhartid = READ_CSR("mhartid");

  printf("Hello world from hart %d: %d\n", mhartid, counter);

  // sleep(1);
}
/* USER CODE END PUC */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Configure the system clock */
  /* USER CODE BEGIN SysInit */
  

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */  
  /* USER CODE BEGIN Init */
  UART_InitType UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART_init_config);

  app_init();
  /* USER CODE END Init */

  benchmark_vec_conditional();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // while (1) {
  //   app_main();
  //   return 0;
  // }
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