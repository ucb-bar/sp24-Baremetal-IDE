// See LICENSE for license details.

//**************************************************************************
// SGEMM benchmark
//--------------------------------------------------------------------------
//
// This benchmark tests a vectorized sgemm implementation.



//--------------------------------------------------------------------------
// Input/Reference Data

#include "dataset1.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "riscv.h"
#include "chip_config.h"
//--------------------------------------------------------------------------
// Input/Reference Data
#include "encoding.h"


//--------------------------------------------------------------------------
// Main
extern volatile uint64_t tohost;
extern volatile uint64_t fromhost;

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

void *vec_igemm_nn (size_t, size_t, size_t, const int*, size_t, const int*, size_t, int*, size_t);

int main( int argc, char* argv[] )
{
  int results_data[ARRAY_SIZE] = {0};

#if PREALLOCATE
  // If needed we preallocate everything in the caches
  vec_igemm_nn(DIM_SIZE, DIM_SIZE, DIM_SIZE, input1_data, DIM_SIZE, input2_data, DIM_SIZE, results_data, DIM_SIZE);
  memset(results_data, 0, sizeof(results_data));
#endif

  // Do the sgemm
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");
 
  vec_igemm_nn(DIM_SIZE, DIM_SIZE, DIM_SIZE, input1_data, DIM_SIZE, input2_data, DIM_SIZE, results_data, DIM_SIZE);
  
  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  printf("mcycle = %lu\r\n", end_time - start_time);
  printf("minstret = %lu\r\n", end_instructions - start_instructions);
  // Check the results
  // return verify( ARRAY_SIZE, results_data, verify_data );
}
