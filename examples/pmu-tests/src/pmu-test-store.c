#include <stdint.h>
#include "riscv.h"
#include "pmu.h"
#define NUM_STORES 1000

static volatile uint64_t store_addr[NUM_STORES];

int main(int argc, char **argv) {
  // enable counter enable
  PMU_COUNTER_ENABLE(3);

  // configure event3 and reset counter3
  printf("---configure event3---\n");
  PMU_EVENT_ENABLE(PMU_EVENT(0, STORE), 3);
  PMU_COUNTER_RESET(3);

  // perform stores
  for (int i = 0; i < NUM_STORES; i++) {
    store_addr[i] = i;
  }
  
  // check event3
  uint64_t counter3_after = PMU_COUNTER_READ(3);
  uint64_t event3_after = READ_CSR("mhpmevent3");
  printf("event3 after: %lx\n", event3_after);
  printf("counter3 after: %lx\n", counter3_after);
}