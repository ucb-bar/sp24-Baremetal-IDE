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

#include <riscv_vector.h>

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

static int verify_double(int n, const volatile int64_t* test, const int64_t* verify)
{
  int i;
  // Unrolled for faster verification
  for (i = 0; i < n/2*2; i+=2)
  {
    int64_t t0 = test[i], t1 = test[i+1];
    int64_t v0 = verify[i], v1 = verify[i+1];
    if (t0 != v0) return i+1;
    if (t1 != v1) return i+2;
  }
  if (n % 2 != 0 && test[n-1] != verify[n-1])
    return n;
  return 0;
}

// Verify the matrices
int verify_matrix(int64_t *matrix, int64_t *golden_matrix, int64_t R, int64_t C) {
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
      if (matrix[c + C * r] != golden_matrix[c + C * r]) {
        printf("Error: o[%d][%d] = %ld, instead of %ld\n", r, c,
               matrix[c + C * r], golden_matrix[c + C * r]);
        return 1;
      }
  return 0;
}

extern uint64_t vsize;
// Vectors for benchmarks
extern int64_t v64a[] __attribute__((aligned(256), section(".l2")));
extern int64_t v64b[] __attribute__((aligned(256), section(".l2")));
extern int32_t v32a[] __attribute__((aligned(256), section(".l2")));
extern int32_t v32b[] __attribute__((aligned(256), section(".l2")));
extern int16_t v16a[] __attribute__((aligned(256), section(".l2")));
extern int16_t v16b[] __attribute__((aligned(256), section(".l2")));
extern int8_t v8a[] __attribute__((aligned(256), section(".l2")));
extern int8_t v8b[] __attribute__((aligned(256), section(".l2")));
// Output vectors
extern int64_t res64_v, res64_s;
extern int32_t res32_v, res32_s;
extern int16_t res16_v, res16_s;
extern int8_t res8_v, res8_s;

void benchmark_vec_dot_product(){

  printf("DOTP %ld\n", vsize);

  unsigned long cycles1, cycles2, instr2, instr1;

  for (uint64_t avl = 8; avl <= vsize; avl *= 8) {
    // Dotp
    printf("Calulating 64b dotp with vectors with length = %lu\r\n", avl);

    // Attempt at Caching the input vectors
    volatile int64_t Ssum64 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Ssum64 += v64a[i] + v64b[i];
    }

    instr1 =  READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res64_s = dotp_s64b(v64a, v64b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Scalar cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);

    // Attempt at Caching the input vectors
    volatile int64_t Vsum64 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Vsum64 += v64a[i] + v64b[i];
    }

    instr1 =  READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res64_v = dotp_v64b(v64a, v64b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Vector cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);

    // if (verify_double(avl, res64_v, res64_s) != 0) {
    //   printf("VECTORIZED output is not correct\n");
    // }
  }

  for (uint64_t avl = 8; avl <= vsize; avl *= 8) {
    // Dotp
    printf("Calulating 32b dotp with vectors with length = %lu\r\n", avl);

    volatile int32_t Ssum32 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Ssum32 += v32a[i] + v32b[i];
    }

    instr1 =  READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res32_s = dotp_s32b(v32a, v32b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Scalar cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);

    volatile int32_t Vsum32 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Vsum32 += v32a[i] + v32b[i];
    }

    instr1 = READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res32_v = dotp_v32b(v32a, v32b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Vector cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);
  }

  for (uint64_t avl = 8; avl <= vsize; avl *= 8) {
    // Dotp
    printf("Calulating 16b dotp with vectors with length = %lu\r\n", avl);

    volatile int16_t Ssum16 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Ssum16 += v16a[i] + v16b[i];
    }

    instr1 =  READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res16_s = dotp_s16b(v16a, v16b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Scalar cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);

    volatile int16_t Vsum16 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Vsum16 += v16a[i] + v16b[i];
    }

    instr1 = READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res16_v = dotp_v16b(v16a, v16b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Vector cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);
  }

  for (uint64_t avl = 8; avl <= vsize; avl *= 8) {
    // Dotp
    printf("Calulating 8b dotp with vectors with length = %lu\r\n", avl);

    volatile int8_t Ssum8 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Ssum8 += v8a[i] + v8b[i];
    }

    instr1 =  READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res8_s = dotp_s8b(v8a, v8b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Scalar cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);

    volatile int8_t Vsum8 = 0;
    for (uint64_t i = 0; i < avl; i += 8) {
      Vsum8 += v8a[i] + v8b[i];
    }

    instr1 = READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    res8_v = dotp_v8b(v8a, v8b, avl);
    asm volatile("fence");
    instr2 = READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");
    printf("Vector cycles: %ld instructions: %ld\r\n", cycles2 - cycles1, instr2 - instr1);
  }
}


// Matrices defined in data.S
extern int64_t i_3x3[] __attribute__((aligned(32))); // [ (M+floor(F/2)) * (N+floor(F/2)) ]
extern int64_t f_3x3[] __attribute__((aligned(32)));        // [ F*F ]
extern int64_t o_3x3[] __attribute__((aligned(32)));        // [ M*N ]
extern int64_t golden_o_3x3[] __attribute__((aligned(32))); // [ M*N ]
// M, N, F defined in data.S
extern int64_t M_3x3;
extern int64_t N_3x3;
extern int64_t F_3x3;

extern int64_t i_5x5[] __attribute__((aligned(32))); // [ (M+floor(F/2)) * (N+floor(F/2)) ]
extern int64_t f_5x5[] __attribute__((aligned(32)));        // [ F*F ]
extern int64_t o_5x5[] __attribute__((aligned(32)));        // [ M*N ]
extern int64_t golden_o_5x5[] __attribute__((aligned(32))); // [ M*N ]
// M, N, F defined in data.S
extern int64_t M_5x5;
extern int64_t N_5x5;
extern int64_t F_5x5;

extern int64_t i_7x7[] __attribute__((aligned(32))); // [ (M+floor(F/2)) * (N+floor(F/2)) ]
extern int64_t f_7x7[] __attribute__((aligned(32)));        // [ F*F ]
extern int64_t o_7x7[] __attribute__((aligned(32)));        // [ M*N ]
extern int64_t golden_o_7x7[] __attribute__((aligned(32))); // [ M*N ]
// M, N, F defined in data.S
extern int64_t M_7x7;
extern int64_t N_7x7;
extern int64_t F_7x7;

void benchmark_vec_iconv(){
  unsigned long cycles1, cycles2, instr2, instr1;
  int64_t runtime;
  int error = 0;

  instr1 = READ_CSR("minstret");
  cycles1 = READ_CSR("mcycle");
  iconv2d_3x3(o_3x3, i_3x3, f_3x3, M_3x3, N_3x3, F_3x3);
  asm volatile("fence");
  instr2 = READ_CSR("minstret");
  cycles2 = READ_CSR("mcycle");

  runtime = cycles2 - cycles1;
  printf("The execution for a 3x3 kernel took %d cycles.\n", runtime);

  error += verify_matrix(o_3x3, golden_o_3x3, M_3x3, N_3x3);

  instr1 = READ_CSR("minstret");
  cycles1 = READ_CSR("mcycle");
  iconv2d_5x5(o_5x5, i_5x5, f_5x5, M_5x5, N_5x5, F_5x5);
  asm volatile("fence");
  instr2 = READ_CSR("minstret");
  cycles2 = READ_CSR("mcycle");

  runtime = cycles2 - cycles1;
  printf("The execution for a 5x5 kernel took %d cycles.\n", runtime);

  error += verify_matrix(o_5x5, golden_o_5x5, M_5x5, N_3x3);

  // Call the main kernel, and measure cycles
  instr1 = READ_CSR("minstret");
  cycles1 = READ_CSR("mcycle");
  iconv2d_7x7(o_7x7, i_7x7, f_7x7, M_7x7, N_7x7, F_7x7);
  asm volatile("fence");
  instr2 = READ_CSR("minstret");
  cycles2 = READ_CSR("mcycle");

  // Performance metrics
  runtime = cycles2 - cycles1;
  printf("The execution for a 7x7 took %d cycles.\n", runtime);

  error += verify_matrix(o_7x7, golden_o_7x7, M_7x7, N_7x7);

  if (error != 0) {
    printf("Fail.\n");
  } else {
    printf("Passed.\n");
  }
}

extern uint64_t M;
extern uint64_t N;
extern uint64_t P;

extern int64_t a[] __attribute__((aligned(256)));
extern int64_t b[] __attribute__((aligned(256)));
extern int64_t c[] __attribute__((aligned(256)));
// Gold results
extern int64_t g[] __attribute__((aligned(256)));

void benchmark_vec_igemm() { 
  printf("IMATMUL\n");
  unsigned long cycles1, cycles2, instr2, instr1;

  for (int s = 4; s <= M; s *= 2) {
    printf("Calculating a (%d x %d) x (%d x %d) matrix multiplication...\n", s,
           s, s, s);

    // Matrices are initialized --> Start calculating
    printf("Calculating imatmul...\n");
    instr1 =  READ_CSR("minstret");
    cycles1 = READ_CSR("mcycle");
    imatmul(c, a, b, s, s, s);
    asm volatile("fence");
    instr2 =  READ_CSR("minstret");
    cycles2 = READ_CSR("mcycle");

    // Metrics
    int64_t runtime = cycles2 - cycles1;
    float performance = 2.0 * s * s * s / runtime;

    printf("The execution took %d cycles.\n", runtime);
    printf("The performance is %ld OPs/1000 cycles.\n", (uint64_t)(1000.0 * performance));

    // Verify the result only for s == M (to keep it simple)
    if (s == M) {
      // Verify the result
      printf("Verifying result...\n");
      int error = verify_matrix(c, g, s, s);
      if (error != 0) {
        printf("Error code %d\n", error);
        printf("c[%d]=%d\n", error, c[error]);
      } else {
        printf("Passed.\n");
      }
    }
  }
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
  /* USER CODycE BEGIN Init */
  UART_InitType UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART_init_config);

  app_init();
  /* USER CODE END Init */

  // benchmark_vec_dot_product();
  benchmark_vec_iconv();
  // benchmark_vec_igemm();

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