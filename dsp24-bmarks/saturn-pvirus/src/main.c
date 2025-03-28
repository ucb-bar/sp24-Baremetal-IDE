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
/* Power virus designed to stress test the saturn core and achieve maximum power consumption
   Expects a payload of 1 32-bit uint specifying how many milliseconds to run the power virus for
   followed by 1 8 bit uint specifying how many HARTs to run the benchmark on.
   Returns no payload
*/

#include "main.h"
#include <riscv_vector.h>
#include "hthread.h"

typedef struct __attribute__((packed)) {
  uint32_t time_ms;
  uint8_t num_harts;
} SaturnMCPayload;

int32_t op1[32];
int32_t op2[32];

void mac_pv_intrinsics(uint64_t mt_cycles) {

  size_t vl = __riscv_vsetvlmax_e8m4();
  vint8m4_t fac1 = __riscv_vle8_v_i8m4((int8_t*) &op1, vl);
  vint8m4_t fac2 = __riscv_vle8_v_i8m4((int8_t*) &op2, vl);

  vint8m4_t acc = __riscv_vle8_v_i8m4((int8_t*) op1, vl);
  vint8m4_t mul = __riscv_vle8_v_i8m4((int8_t*) op1, vl);

  uint64_t start_time = get_cycles();
  uint64_t target_cycles = start_time + mt_cycles;

  while(get_cycles() < target_cycles) {
    acc = __riscv_vmacc_vv_i8m4(acc, fac1, fac2, vl);
    mul = __riscv_vdiv_vv_i8m4(fac1, fac2, vl);
  }
  volatile vint8m4_t result1 = acc;
  volatile vint8m4_t result2 = mul;
}

void mac_pv_mh(uint8_t num_harts, uint64_t mt_cycles) {
  srand(0xdeadbeef);
  
  uint32_t* clock_gaters = 0x100004;
  for (int i = num_harts; i < 4; i++) {
    clock_gaters[i] = 0;
  }

  for (int i = 0; i < 32; i++) {
    op1[i] = rand();
    op2[i] = rand();
  }

  start_roi();
  for (int i = 1; i < num_harts; i++) {
    hthread_issue(i, &mac_pv_intrinsics, mt_cycles);
  }
  mac_pv_intrinsics(mt_cycles);
  for (int i = 1; i < num_harts; i++) {
    hthread_join(i);
  }

  end_roi();
  xmit_payload_packet(NULL, 0);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  while (1) {
    test_info t = init_test(UART1);
    uint8_t harts = ((SaturnMCPayload*) &t.payload)->num_harts;
    uint64_t cycles = ((SaturnMCPayload*) &t.payload)->time_ms * chip_freq / 500;

    switch (t.testid) {
      case 0:
        mac_pv_mh(harts, cycles);
        break;
      // case 1:
      //   mac_pv_asm(cycles);
    }
    clean_test(t);
    *((uint32_t*) 0x100008) = 1;
    *((uint32_t*) 0x10000C) = 1;
    *((uint32_t*) 0x100010) = 1;
  }
}