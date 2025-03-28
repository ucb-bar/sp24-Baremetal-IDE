#include "syscalls.h"

// void init_heap (void *heap, size_t heap_size) {
//   // assert(heap_size % sizeof(void *) == 0);  /* see #138 */
//   heap_ptr = (void *) heap;
//   heap_end = (void *) ((char *) heap_ptr + heap_size);
//   heap_requested = 0;
// }

/* BEEBS version of malloc.

   This is primarily to reduce library and OS dependencies. Malloc is
   generally not used in embedded code, or if it is, only in well defined
   contexts to pre-allocate a fixed amount of memory. So this simplistic
   implementation is just fine.

   Note in particular the assumption that memory will never be freed! */

// void *malloc (size_t size) {
//   if (size == 0)
//     return NULL;

//   void *next_heap_ptr = (char *)heap_ptr + size;

//   heap_requested += size;

//   const size_t alignment = sizeof (void *);

//   /* Check if the next heap pointer is aligned, otherwise add some padding */
//   if (((uintptr_t)next_heap_ptr % alignment) != 0)
//     {
//       size_t padding = alignment - ((uintptr_t)next_heap_ptr % alignment);

//       next_heap_ptr = (char *)next_heap_ptr + padding;

//       /* padding is added to heap_requested because otherwise it will break
//          check_heap_beebs() */
//       heap_requested += padding;
//     }

//   /* Check if we can "allocate" enough space */
//   if (next_heap_ptr > heap_end)
//     return NULL;

//   void *new_ptr = heap_ptr;
//   heap_ptr = next_heap_ptr;

//   return new_ptr;
// }


// /* BEEBS version of calloc.

//    Implement as wrapper for malloc */

// void *calloc (size_t nmemb, size_t size) {
//   void *new_ptr = malloc (nmemb * size);

//   /* Calloc is defined to zero the memory. OK to use a function here, because
//      it will be handled specially by the compiler anyway. */

//   if (NULL != new_ptr)
//     memset (new_ptr, 0, nmemb * size);

//   return new_ptr;
// }


// /* BEEBS version of realloc.

//    This is primarily to reduce library and OS dependencies. We just have to
//    allocate new memory and copy stuff across. */

// void *realloc (void *ptr, size_t size) {
//   if (ptr == NULL)
//     return NULL;

//   /* Get a new aligned pointer */
//   void *new_ptr = malloc (size);

//   /* This is clunky, since we don't know the size of the original pointer.
//      However it is a read only action and we know it must be big enough if we
//      right off the end, or we couldn't have allocated here. If the size is
//      smaller, it doesn't matter. */

//   if (new_ptr != NULL)
//     for (size_t i = 0; i < size; i++)
//       ((char *)new_ptr)[i] = ((char *)ptr)[i];

//   return new_ptr;
// }


// /* For our simplified version of memory handling, free can just do nothing. */
// void free (void *ptr __attribute__ ((unused))) {}



ssize_t _write(int fd, const void *ptr, size_t len) {
  uart_transmit(UART0, (const uint8_t *)ptr, len, 100);
  return 0;
}

extern char __end[];
static char *curbrk = __end;

void *_sbrk(ptrdiff_t incr) {
  extern char __heap_end[];
  char *newbrk;
  char *oldbrk;

  oldbrk = curbrk;
  newbrk = oldbrk + incr;
  if (unlikely((newbrk < __end) || (newbrk >= __heap_end))) {
    errno = ENOMEM;
    return (void *)(-1);
  }

  curbrk = newbrk;
  return oldbrk;
}

int _fstat(int file, struct stat *st) {
  errno = EBADF;
  return -1;
}
