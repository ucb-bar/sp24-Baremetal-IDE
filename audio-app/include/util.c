#include <stddef.h>
#include <stdint.h>

double compute_pi(int iterations) {
    double pi = 0.0;
    int sign = 1;

    for (long long i = 0; i < iterations; i++) {
        pi += sign * (1.0 / (2 * i + 1));
        sign = -sign;
    }

    pi *= 4;

    return pi;
}

void software_conv(int8_t *arr, size_t arr_len, short* kernel, size_t kernel_len, size_t dilation, short* output) {
  for (int i = 0; i < arr_len; i += 1) {
    
    output[i] = 0;
    for (int j = 0; j < kernel_len; j += 1) {
      int arr_index = i + j * dilation;
      int8_t item;
      if (arr_index >= arr_len) {
        item = 0;
      } else {
        item = arr[arr_index];
      }
      output[i] = f16_add(output[i], f16_mul(f16_from_int((int32_t)item), kernel[j]));
    }
  }
}