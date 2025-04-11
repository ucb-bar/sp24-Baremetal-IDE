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
// Add => Normalize => Play it back?
#include "main.h"

typedef struct __attribute__((__packed__)) {
  uint64_t naive_cycles;
  float naive_performance;
  uint64_t vector_cycles;
  float vector_performance;
} igemm_result_t;

extern void imatmul(int64_t *c, const int64_t *a, const int64_t *b,
  const unsigned long int m, const unsigned long int n,
  const unsigned long int p);

extern uint64_t M_big;
extern uint64_t N_big;
extern uint64_t P_big;

extern int64_t a_big[] __attribute__((aligned(256)));
extern int64_t b_big[] __attribute__((aligned(256)));
extern int64_t c_big[] __attribute__((aligned(256)));
// Gold results
extern int64_t g_big[] __attribute__((aligned(256)));
  
#define TILE 8

void benchmark_naive_igemm_big(){
  int s = 64;

  igemm_result_t result;
  memset(&result, 0, sizeof(result));

  uint64_t runtime;
  float performance;
  uint64_t time;

  for (int rep = 0; rep < 3; rep++) {
    if (rep == 2) { 
      start_roi();
      time = READ_CSR("mcycle");
    }
    for (uint64_t i0 = 0; i0 < s; i0 += TILE) {
      for (uint64_t j0 = 0; j0 < s; j0 += TILE) {
        for (uint64_t k0 = 0; k0 < s; k0 += TILE) {
    
          for (uint64_t i = i0; i < i0 + TILE && i < s; i++) {
            for (uint64_t j = j0; j < j0 + TILE && j < s; j++) {
              int64_t sum = c_big[i * s + j]; 
    
              for (uint64_t k = k0; k < k0 + TILE && k < s; k++) {
                sum += a_big[i * s + k] * b_big[k * s + j];
              }
    
              c_big[i * s + j] = sum;
            }
          }
        }
      }
    }

    if (rep == 2){
      runtime = READ_CSR("mcycle") - time;
      end_roi();
      result.naive_cycles = runtime;
      result.naive_performance = 2.0 * s * s * s / runtime;
      xmit_payload_packet(&result, 24);
    }
  }
}

void benchmark_vec_igemm_big(){
  int s = 256;

  igemm_result_t result;
  memset(&result, 0, sizeof(result));

  uint64_t runtime;
  uint64_t time;

  for (int rep = 0; rep < 4; rep++) {
    if (rep == 3) { 
      start_roi();
      time = READ_CSR("mcycle");
    }

    imatmul(c_big, a_big, b_big, s, s, s);

    asm volatile("fence");

    if (rep == 3){
      runtime = READ_CSR("mcycle") - time;
      end_roi();
      result.vector_cycles = runtime;
      result.vector_performance = 2.0 * s * s * s / runtime;
      xmit_payload_packet(&result, 24);
    }
  }
}

void benchmark_vec_naive(){

  int s = 64;

  igemm_result_t result;
  memset(&result, 0, sizeof(result));

  uint64_t runtime;
  float performance;
  
  uint64_t time;
  
  for (int rep = 0; rep < 3; rep++) {
    if (rep == 2) { 
      time = READ_CSR("mcycle");
    }
    for (uint64_t i0 = 0; i0 < s; i0 += TILE) {
      for (uint64_t j0 = 0; j0 < s; j0 += TILE) {
        for (uint64_t k0 = 0; k0 < s; k0 += TILE) {
    
          for (uint64_t i = i0; i < i0 + TILE && i < s; i++) {
            for (uint64_t j = j0; j < j0 + TILE && j < s; j++) {
              int64_t sum = c_big[i * s + j]; 
    
              for (uint64_t k = k0; k < k0 + TILE && k < s; k++) {
                sum += a_big[i * s + k] * b_big[k * s + j];
              }
    
              c_big[i * s + j] = sum;
            }
          }
        }
      }
    }

    if (rep == 2){
      runtime = READ_CSR("mcycle") - time;
      result.naive_cycles = runtime;
      result.naive_performance = 2.0 * s * s * s / runtime;
    }
  }

  for (int rep = 0; rep < 4; rep++) {
    if (rep == 3) { 
      start_roi();
      time = READ_CSR("mcycle");
    }
    
    imatmul(c_big, a_big, b_big, s, s, s);

    asm volatile("fence");

    if (rep == 3){
      runtime = READ_CSR("mcycle") - time;
      end_roi();
      result.vector_cycles = runtime;
      result.vector_performance = 2.0 * s * s * s / runtime;
      xmit_payload_packet(&result, 24);
    }
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  while (1) {
    test_info t = init_test(UART1);
    switch (t.testid) {
      case 0:
        benchmark_naive_igemm_big();
        break;
      case 1:
        benchmark_vec_igemm_big();
        break;
      case 2: 
        benchmark_vec_naive();
        break;
      default:
        benchmark_vec_naive();
        break;
    }
  }
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