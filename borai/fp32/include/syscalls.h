#include <sys/stat.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

#include "hardware.h"

#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// static void *heap_ptr = ((volatile void*)DRAM_BASE);
// static void *heap_end = ((volatile void*)DRAM_BASE + 0x06000000U);
// static size_t heap_requested = 0;

// void init_heap (void *heap, size_t heap_size);
// void *malloc (size_t size);
// void *calloc (size_t nmemb, size_t size);
// void *realloc (void *ptr, size_t size);
// void free (void *ptr __attribute__ ((unused)));
