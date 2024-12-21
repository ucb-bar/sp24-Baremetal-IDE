#ifndef BEARLYCONV_H
#define BEARLYCONV_H

#include <stdint.h>

void hardware_convolution(int8_t *source, int16_t *dest, int width, int height, int8_t *kernel);
void software_convolution(int8_t *source, int16_t *dest, int width, int height, int8_t *kernel, int start_col, int end_col);
void convolution(int8_t *source, int16_t *dest, int width, int height, int8_t *kernel);

#endif // BEARLYCONV_H

