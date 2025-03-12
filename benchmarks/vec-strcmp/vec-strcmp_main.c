// See LICENSE for license details.

//**************************************************************************
// Strcmp benchmark
//--------------------------------------------------------------------------
//
// This benchmark tests a vectorized strcmp implementation.

#include <string.h>
#include "util.h"


#include "riscv.h"
#include "chip_config.h"
//--------------------------------------------------------------------------
// Input/Reference Data
#include "encoding.h"

//--------------------------------------------------------------------------
// Input/Reference Data

const char* test_str = "The quick brown fox jumped over the lazy dog";
const char* same_str = "The quick brown fox jumped over the lazy dog";
char* diff_str = "The quick brown fox jumped over the lazy cat";

//--------------------------------------------------------------------------
// Main

int vec_strcmp(const char *src1, const char* src2);

int main( int argc, char* argv[] )
{
  // Do the strcmp
  uint64_t start_time = READ_CSR("mcycle");
  uint64_t start_instructions = READ_CSR("minstret");
  int r0 = vec_strcmp(test_str, same_str);
  int r1 = vec_strcmp(test_str, diff_str);
  uint64_t end_time = READ_CSR("mcycle");
  uint64_t end_instructions = READ_CSR("minstret");

  printf("mcycle = %lu\r\n", end_time - start_time);
  printf("minstret = %lu\r\n", end_instructions - start_instructions);

  // Check the results
  return !(r0 == 0 && r1 != 0);
}
