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

/* Benchmarks to test the memcpy/sbus bandwidth of the chip using naive CPU code, glibc, rvv, and the DSP24 DMA Engine
   Expects a 32 bit int payload for the random seed for each iteration and returns a packet containing 1 64 bit unsigned
   integer which is the number of cycles used and a byte which indicates whether or not the copy occured correctly.
*/
#include "main.h"
#include <riscv_vector.h>

uint32_t TEST_SIZE = 0x1000;
void* srcbuf = 0xBFFE0000;
void* dstbuf = 0xBFFF0000;

typedef struct {
  uint64_t cycles;
  bool correct;
} memcpy_result_t;

void init_buffer(volatile uint32_t* buf, int size, int seed) {
  srand(seed);
  for (int i = 0; i < size; i++) {
    buf[i] = rand();
  }
}

bool check_buffer(volatile uint32_t* buf, int size, int seed) {
  srand(seed);
  for (int i = 0; i < size; i++) {
    if(buf[i] != rand()) {
      return false;
    }
  }
  return true;
}

void touch_buffer(volatile uint8_t* buf, int size) {
  volatile uint8_t var;
  for (int i = 0; i < size; i += 16) {
    var = buf[i];
  }
}

void func_test(int seed) {
  memcpy_result_t result;
  uint64_t time;
  init_buffer(srcbuf, TEST_SIZE/4, seed);
  touch_buffer(dstbuf, TEST_SIZE);

  start_roi();
  result.cycles = 431987423;
  end_roi();
  result.correct = check_buffer(srcbuf, TEST_SIZE/4, seed);
   xmit_payload_packet(&result, 9);

}

void cpu_memcpy(int seed) {
  memcpy_result_t result;
  uint64_t time;
  init_buffer(srcbuf, TEST_SIZE/4, seed);
  touch_buffer(dstbuf, TEST_SIZE);

  start_roi();
  time = get_cycles();
  volatile uint64_t* src = srcbuf;
  volatile uint64_t* dst = dstbuf;

  for (int i = 0; i < TEST_SIZE/8; i++) {
    dst[i] = src[i];
  }
  result.cycles = get_cycles() - time;
  end_roi();
  result.correct = check_buffer(dstbuf, TEST_SIZE/4, seed);
   xmit_payload_packet(&result, 9);
}

void glibc_memcpy(int seed) {
  memcpy_result_t result;
  uint64_t time;
  init_buffer(srcbuf, TEST_SIZE/4, seed);
  touch_buffer(dstbuf, TEST_SIZE);

  start_roi();
  time = get_cycles();
  memcpy(dstbuf, srcbuf, TEST_SIZE);
  result.cycles = get_cycles() - time;
  end_roi();
  result.correct = check_buffer(dstbuf, TEST_SIZE/4, seed);
   xmit_payload_packet(&result, 9);
}

void rvv_memcpy(int seed) {
  memcpy_result_t result;
  uint64_t time;

  void* srcbuf_ptr = srcbuf;
  void* dstbuf_ptr = dstbuf;
  uint32_t remaining = TEST_SIZE;

  init_buffer(srcbuf, TEST_SIZE/4, seed);
  touch_buffer(dstbuf, TEST_SIZE);

  start_roi();
  time = get_cycles();

  for (size_t vl; remaining > 0; remaining -= vl, srcbuf_ptr += vl, dstbuf_ptr += vl) {
    vl = __riscv_vsetvl_e8m8(TEST_SIZE);

    vuint8m8_t vec_src = __riscv_vle8_v_u8m8(srcbuf_ptr, vl);
    __riscv_vse8_v_u8m8(dstbuf_ptr, vec_src, vl);
  }
  
  result.cycles = get_cycles() - time;
  end_roi();
  result.correct = check_buffer(dstbuf, TEST_SIZE/4, seed);
   xmit_payload_packet(&result, 9);
}

void dma_memcpy(int seed) {
  memcpy_result_t result;
  uint64_t time;
  init_buffer(srcbuf, TEST_SIZE/4, seed);
  touch_buffer(dstbuf, TEST_SIZE);

  start_roi();
  time = get_cycles();
  enable_Crack();
  set_DMAC(0, srcbuf, dstbuf, 64, 64, 1024, 3);
  start_DMA(0);
  while (*(volatile char*) (DMA_BASE+0x1) != 0);
  result.cycles = get_cycles() - time;
  end_roi();
  result.correct = check_buffer(dstbuf, TEST_SIZE/4, seed);
  xmit_payload_packet(&result, 9);
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  while (1) {
    test_info t = init_test(UART1);
    int seed = *((int*) &t.payload);
    switch (t.testid) {
      case 0:
        cpu_memcpy(seed);
        break;
      case 1:
        glibc_memcpy(seed);
        break;
      case 2:
        rvv_memcpy(seed);
        break;
      case 3:
        dma_memcpy(seed);
        break;
      default:
        func_test(seed);
        break;
    }
    clean_test(t);
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