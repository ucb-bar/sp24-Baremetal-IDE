#include "time.h"
#include "libbmark.h"
#include "chip_config.h"

unsigned int sleep(unsigned int seconds) {
  uint64_t target_tick = clint_get_time((CLINT_Type *)CLINT_BASE) + (seconds * chip_mtime_freq);
  
  while (clint_get_time((CLINT_Type *)CLINT_BASE) < chip_freq/1000) {
    asm volatile("nop");
  }
  return 0;
}

int msleep(useconds_t msec) {
  uint64_t target_tick = clint_get_time((CLINT_Type *)CLINT_BASE) + ((msec * chip_mtime_freq) / 1000);
  
  while (clint_get_time((CLINT_Type *)CLINT_BASE) < target_tick) {
    asm volatile("nop");
  }
  return 0;
}

int usleep(useconds_t usec) {
  uint64_t target_tick = clint_get_time((CLINT_Type *)CLINT_BASE) + ((usec * chip_mtime_freq) / 1000000);
  
  while (clint_get_time((CLINT_Type *)CLINT_BASE) < target_tick) {
    asm volatile("nop");
  }
  return 0;
}
