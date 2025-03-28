/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Jasmine Angle (angle@berkeley.edu)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * Number of elements contained within a single quantized transformer vector.
 */
#define VECTOR_SIZE 8


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

/**
 * run_dotprod_test
 * 
 * Performs a sweep of multiple ways for using vec_1 and vec_2 as inputs to the
 * Quantized Transformer's V_DOTPROD function. This function is not pure,
 * though prints out results to UART0, assuming a valid UART channel has been
 * initialized prior to execution.
 * 
 * Inputs:
 *  int8_t *vec_1: Contents of the second dot product vector (8 elements).
 *  int8_t *vec_2: Contents of the second dot product vector (8 elements).
 *  int8_t scale0: Scale factor param 0.
 *  int8_t scale1: Scale factor param 1.
 *  char *test_name: Name for the test to print to UART0.
 */
void run_dotprod_test(int8_t *vec_1, int8_t *vec_2, int8_t scale0, int8_t scale1, char* test_name) {
  uint64_t mhartid = READ_CSR("mhartid");
  size_t i = 0;
  int8_t* vec_out_1 = malloc(VECTOR_SIZE * sizeof(int8_t));
  int8_t* vec_out_2 = malloc(VECTOR_SIZE * sizeof(int8_t));
  
  printf("--------------- [HART ID %lu] Starting test \"%s\" ---------------\r\n", mhartid, test_name);
  printf("[GIVEN]\r\n");
  printf("Vector 1:");
  for (i = 0; i < VECTOR_SIZE; i++) {
    printf("\t%d", (int8_t) vec_1[i]);
  }
  printf("\r\n");

  printf("Vector 2:");
  for (i = 0; i < VECTOR_SIZE; i++) {
    printf("\t%d", (int8_t) vec_2[i]);
  }
  printf("\r\n");

  printf("Loading vectors...");
  asm volatile("fence");
  V_LOAD(1, vec_1);
  asm volatile("fence");

  asm volatile("fence");
  V_LOAD(2, vec_2);
  asm volatile("fence");
  printf("Done!\r\n");

  // Move vec_1 from accelerator to CPU
  printf("Retrieving QT vectors for verification...");
  asm volatile("fence");
  V_STORE(1, vec_out_1);
  asm volatile("fence");

  // Move vec_2 from accelerator to CPU
  asm volatile("fence");
  V_STORE(2, vec_out_2);
  asm volatile("fence");
  printf("Done!\r\n");

  printf("Stored 1:");  
  for (i = 0; i < VECTOR_SIZE; i++) {
    printf("\t%d", (int8_t) vec_out_1[i]);
    if (vec_out_1[i] != vec_1[i]) {
      printf("[!]");
    }
  }
  printf("\r\n");

  printf("Stored 2:");
  for (i = 0; i < VECTOR_SIZE; i++) {
    printf("\t%d", (int8_t) vec_out_2[i]);
    if (vec_out_2[i] != vec_2[i]) {
      printf("[!]");
    }
  }
  printf("\r\n");


  printf("Generating naive solution:\r\n");
  int16_t naive_dotprod_result = 0;
  for (i = 0; i < VECTOR_SIZE; i++) {
    int16_t mul_res = ((int8_t) vec_1[i]) * ((int8_t) vec_2[i]);
    naive_dotprod_result += mul_res;
    printf("\t[Element %lu] %d = %d * %d (Cumulative sum is now %d)\r\n",
      i, mul_res, vec_1[i], vec_2[i], naive_dotprod_result);
  }
  printf("Naive Expected Output: %d\r\n", naive_dotprod_result);

  printf("Setting scale factor of %d, %d...", scale0, scale1);
  asm volatile("fence");
  SET_SCALE_FACTOR(scale0, scale1);
  asm volatile("fence");
  printf("Done!\r\n");

  printf("[WHEN]\r\nDot product is executed...\r\n");
  int16_t rd;
  asm volatile("fence");
  V_DOT_PROD(rd, 1, 2);
  asm volatile("fence");
  printf("Done!\r\nV_DOTPROD destination register equals %d (%#2x)\r\n", (int16_t)rd, (int16_t)rd);

  printf("[THEN]\r\nThe vectorized dot product should equal the naive result...");
  if (naive_dotprod_result != rd) {
    printf("FAIL! <----\r\n");
  } else {
    printf("Pass!\r\n");
  }

  printf("--------------- [HART ID %lu] Finished test \"%s\" ---------------\r\n", mhartid, test_name);
}


void app_init() {
  // torch::executor::runtime_init();
}



void app_main() {
  // Clears the entire screen and moves character pointer to top left
  printf("%c%c%c%c%c[0;0H",0x1B,0x5B,0x32,0x4A,0x1B);
  //while(1) {

    int8_t v1_1[VECTOR_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
    int8_t v2_1[VECTOR_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1};
    run_dotprod_test(v1_1, v2_1, 1, 0, "Simple multiply unsigned by 1");

    int8_t v1_2[VECTOR_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
    int8_t v2_2[VECTOR_SIZE] = {1, 1, 3, 4, 5, 6, 7, 8};
    run_dotprod_test(v1_2, v2_2, 1, 0, "Multiply incremented unsigned values");

    int8_t v1_3[VECTOR_SIZE] = {53, 127, 0, 62, 123, 151, 77, 15};
    int8_t v2_3[VECTOR_SIZE] = {37, 65, 67, 81, 126, 130, 8, 55};
    run_dotprod_test(v1_3, v2_3, 1, 0, "Unordered int8 unsigned values");

    int8_t v1_4[VECTOR_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
    int8_t v2_4[VECTOR_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};
    run_dotprod_test(v1_4, v2_4, 1, 0, "Simple multiply by 0 test");

    int8_t v1_5[VECTOR_SIZE] = {-1, -2, -3, -4, -5, -6, -7, -8};
    int8_t v2_5[VECTOR_SIZE] = {1, 1, 1, 1, 1, 1, 1, 1};
    run_dotprod_test(v1_5, v2_5, 1, 0, "Simple multiply signed by 1");

    int8_t v1_6[VECTOR_SIZE] = {-1, -2, -3, -4, -5, -6, -7, -8};
    int8_t v2_6[VECTOR_SIZE] = {1, 0, 0, 0, 0, 0, 0, 0};
    run_dotprod_test(v1_6, v2_6, 1, 0, "Simple multiply all zero except one signed");

    // printf("starting LOAD STORE test\r\n");
    // int vec_size = 8;
    // int8_t* vec_1 = malloc(vec_size * sizeof(int8_t));
    // int8_t* vec_2 = malloc(vec_size * sizeof(int8_t));
    // int8_t* vec_out_1 = malloc(vec_size * sizeof(int8_t));
    // int8_t* vec_out_2 = malloc(vec_size * sizeof(int8_t));
    // int8_t* vec_out_3 = malloc(vec_size * sizeof(int8_t));

    // int8_t* softmax_vec_in = malloc(vec_size * sizeof(int8_t));
    // int8_t* softmax_vec_out = malloc(vec_size * sizeof(int8_t));

    
    // // Initialize vec_1 and vec_2
    // for (int i = 0; i < vec_size; i++) {
    //     vec_1[i] = (int8_t) i;
    //     //vec_2[i] = (int8_t) 0;
    //     // vec_2[i] = (int8_t) (-i);
    //     vec_2[i] = (int8_t) (-1);
    // }
    // //vec_1[0] = (int8_t) 3;
    // //vec_2[0] = (int8_t) 1;

    // // printf("vec_1:\n");
    // // for (int i = 0; i < vec_size; i++) {
    // //     printf("vec_1[%d]: %d\n", i, vec_1[i]);
    // // }

    // // printf("\nvec_2:\n");
    // // for (int i = 0; i < vec_size; i++) {
    // //     printf("vec_2[%d]: %d\n", i, vec_2[i]);
    // // }

    // // Load vec_1 and vec_2 into accelerator
    // asm volatile("fence");
    // V_LOAD(1, vec_1);
    // asm volatile("fence");

    // asm volatile("fence");
    // V_LOAD(2, vec_2);
    // asm volatile("fence");

    // asm volatile("fence");
    // V_LOAD(3, vec_2);
    // asm volatile("fence");

    // // Move vec_1 from accelerator to CPU
    // asm volatile("fence");
    // V_STORE(1, vec_out_1);
    // asm volatile("fence");

    // printf("\r\nvec_out:\r\n");
    // for (int i = 0; i < vec_size; i++) {
    //     printf("Expecting loaded result %d in vec_1 to be %d, is: %d\r\n", i, vec_1[i], vec_out_1[i]);
    // }

    // // Move vec_2 from accelerator to CPU
    // asm volatile("fence");
    // V_STORE(2, vec_out_2);
    // asm volatile("fence");

    // printf("vec_out:\r\n");
    // for (int i = 0; i < vec_size; i++) {
    //     printf("Expecting loaded result %d in vec_2 to be %d, is: %d\r\n", i, vec_2[i], vec_out_2[i]);
    // }

    // // // Move vec_2 from accelerator to CPU into vec3
    // // asm volatile("fence");
    // // V_STORE(3, vec_out_3);
    // // asm volatile("fence");

    // // printf("vec_out:\r\n");
    // // for (int i = 0; i < vec_size; i++) {
    // //     printf("Expecting loaded result %d in vec_3 to be %d, is: %d\r\n", i, i - 25, vec_out_3[i]);
    // // }

    // printf("Set scale factor to 1 0\r\n");
    // asm volatile("fence");
    // SET_SCALE_FACTOR(1, 0);
    // asm volatile("fence");

    // printf("Dot product:\r\n");
    // asm volatile("fence");
    // uint64_t rd;
    // V_DOT_PROD(rd, 1, 2);
    // asm volatile("fence");

    // printf("rd: %" PRId64 "\r\n", rd);

    // int16_t naive_dotprod_result = 0;
    // for (int i = 0; i < vec_size; i++) {
    //   int16_t mul_res = ((int8_t) vec_1[i]) * ((int8_t) vec_2[i]);
    //   naive_dotprod_result += mul_res;
    //   printf("\tSumming %d = %d * %d (naive_dotprod_result now %d)\r\n", mul_res, vec_1[i], vec_2[i], naive_dotprod_result);
    // }
    // printf("Naive Output: %d, QT Output: %d\r\n", naive_dotprod_result, rd);

    // // printf("\nReLU:\n");
    // // asm volatile("fence");
    // // V_RELU(2);
    // // asm volatile("fence");

    // // printf("Softmax:\n");
    // // asm volatile("fence");
    // // V_SOFTMAX(2);
    // // asm volatile("fence");

    // // Move vec_2 from accelerator to CPU
    // // asm volatile("fence");
    // // V_STORE(0, vec_out_1);
    // // asm volatile("fence");

    // // printf("vec_out_1:\n");
    // // for (int i = 0; i < vec_size; i++) {
    // //     printf("vec_1[%d]: %d\n", i, vec_out_1[i]);
    // // }

    // // printf("LAYER_NORM:\r\n");
    // // asm volatile("fence");
    // // V_LAYERNORM(2);
    // // asm volatile("fence");

    // // Move vec_2 from accelerator to CPU
    // // asm volatile("fence");
    // // V_STORE(0, vec_out_2);
    // // asm volatile("fence");
    // // printf("Printing vec2 original:\r\n");
    // // for (int i = 0; i < vec_size; i++) {
    // //     printf("%d ", vec_2[i]);
    // // }
    // // printf("\r\n");
    // // printf("Printing stored vec2 from transformer:\r\n");
    // // for (int i = 0; i < vec_size; i++) {
    // //     printf("%d ", vec_out_2[i]);
    // // }
    // printf("\r\n");

    // printf("DONE\r\n\r\n");

  //}
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