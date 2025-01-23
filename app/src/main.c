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
#include "main.h"
#include "chip_config.h"
#include "hal_conv.h"
#include <math.h>
#include "test_image.h"
// #include "../../../tests/mmio.h"
// #include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265359
#define HEIGHT 240
#define WIDTH 320
#define MAX_KERNEL_SIZE 31  // This will support sigma up to 5.0

#define INPUT_ADDR      0x08800000
#define OUTPUT_ADDR     0x08800020
#define KERNEL_ADDR     0x08800040
#define START_ADDR      0x0880006C
#define LENGTH_ADDR     0x08800078
#define DILATION_ADDR   0x0880007C
#define INPUT_TYPE_ADDR 0x0880008E
#define RESET_ADDR      0x0880008E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN PUC */


void app_init() {
  // torch::executor::runtime_init();
}

void calculate_pwm_to_center(
    int32_t cx,
    int32_t cy
) {
    

    // Define duty cycle ranges for idx (x-axis) and idy (y-axis)
    const uint32_t duty_x_min = 40;
    const uint32_t duty_x_max = 60;
    const uint32_t duty_y_min = 27;
    const uint32_t duty_y_max = 50;

    // Screen center
    const int32_t screen_center_x = 160;
    const int32_t screen_center_y = 120;

    // Calculate offsets from center
    int32_t offset_x = cx - screen_center_x;
    int32_t offset_y = cy - screen_center_y;

    // Map offsets to duty cycle ranges
    // X-axis: Scale offset to 40%-60% range (center is 50%)
    uint32_t duty_x = 50 + (offset_x * 10) / screen_center_x;

    // Y-axis: Scale offset to 27%-50% range (center is 38.5%)
    uint32_t duty_y = 38.5 + (offset_y * 11.5) / screen_center_y;

    // Clamp the duty cycles to their valid ranges
    if (duty_x < duty_x_min) duty_x = duty_x_min;
    if (duty_x > duty_x_max) duty_x = duty_x_max;

    if (duty_y < duty_y_min) duty_y = duty_y_min;
    if (duty_y > duty_y_max) duty_y = duty_y_max;

    // Apply the calculated duty cycles
    pwm_set_duty_cycle(PWM0_BASE, 1, duty_x, 400, 0);
    pwm_set_duty_cycle(PWM0_BASE, 2, duty_y, 400, 0);
    return;
}


int my_strcmp(const char *str1, const char *str2) {
    while (*str1 && *str2) {
        if (*str1 != *str2) {
            return (unsigned char)*str1 - (unsigned char)*str2;
        }
        str1++;
        str2++;
    }
    return (unsigned char)*str1 - (unsigned char)*str2;
}

void read_pixels(uint8_t* image_data) { 
    uart_receive(UART1, image_data, WIDTH*HEIGHT, 100000);

    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        image_data[i] = (int) image_data[i];
    }
}

void capture_image(uint8_t* image_data) {

    // printf("r\n");
    char* r = "r\n";
    uart_transmit(UART1, (const uint8_t *) r, 2*sizeof(char), 1000);
    // printf("x\n");
    char* x = "x\n";
    uart_transmit(UART1, (const uint8_t *) x, 2*sizeof(char), 1000);

    int index = 0;
    int start_found = 0; // false
    char received; 
    char buffer[60];

    while (1) {
        uart_receive(UART1, &received, sizeof(char), 100000);
        if (received == '\n' || index >= sizeof(buffer) - 1) {
            buffer[index] = '\0';
            index = 0;

            if (my_strcmp(buffer, "START\r") == 0) {
                uint64_t cpu_start_cycles = READ_CSR("mcycle");
                read_pixels(image_data);
                uint64_t cpu_end_cycles = READ_CSR("mcycle");
                volatile uint64_t diff = cpu_end_cycles - cpu_start_cycles;
            } else if (my_strcmp(buffer, "END\r") == 0) {
                break;
            }
        } else {
            buffer[index++] = received;
        }
    }
}

// Double threshold implementation
void doubleThreshold_opt(uint8_t* src, uint8_t* dst, int size, float highThreshold, float lowThreshold) {
    const uint8_t high = highThreshold * 255;
    const uint8_t low = lowThreshold * 255;
    register uint8_t strong_edge = 255;
    register uint8_t weak_edge = 128;
    
    uint8_t* const src_end = src + size;
    
    while (src + 16 < src_end) {
        asm volatile(
            "vsetvli zero, %0, e8, m8, ta, ma\n\t"
            "vle8.v v8, (%1)\n\t"
            // Compare with high threshold into v0
            "vmsgeu.vx v0, v8, %2\n\t"         
            // Initialize and set strong edges
            "vmv.v.x v16, x0\n\t"              
            "vmerge.vxm v16, v16, %3, v0\n\t"  
            // Compare with low threshold
            "vmsgeu.vx v1, v8, %4\n\t"         
            // Save strong edge mask
            "vmv.v.v v2, v0\n\t"               
            // Set v0 = low AND NOT high directly
            "vmandn.mm v0, v1, v2\n\t"         
            // Set weak edges
            "vmerge.vxm v16, v16, %5, v0\n\t"  
            // Store results
            "vse8.v v16, (%6)\n\t"
            "vle8.v v8, (%7)"                  
            :
            : "r"(16), "r"(src), 
              "r"(high), "r"(strong_edge), 
              "r"(low), "r"(weak_edge), 
              "r"(dst), "r"(src + 16)
            : "v0", "v1", "v2", "v8", "v16", "memory"
        );

        src += 16;
        dst += 16;
    }

    // Handle remaining elements
    if (src < src_end) {
        size_t vl;
        asm volatile(
            "vsetvli %0, %1, e8, m8, ta, ma\n\t"
            "vle8.v v8, (%2)\n\t"
            // Compare with high threshold into v0
            "vmsgeu.vx v0, v8, %3\n\t"
            // Initialize and set strong edges
            "vmv.v.x v16, x0\n\t"
            "vmerge.vxm v16, v16, %4, v0\n\t"
            // Compare with low threshold
            "vmsgeu.vx v1, v8, %5\n\t"
            // Save strong edge mask
            "vmv.v.v v2, v0\n\t"
            // Set v0 = low AND NOT high directly
            "vmandn.mm v0, v1, v2\n\t"
            // Set weak edges
            "vmerge.vxm v16, v16, %6, v0\n\t"
            // Store results
            "vse8.v v16, (%7)"
            : "=r"(vl)
            : "r"((size_t)(src_end - src)), "r"(src),
              "r"(high), "r"(strong_edge),
              "r"(low), "r"(weak_edge), "r"(dst)
            : "v0", "v1", "v2", "v8", "v16", "memory"
        );
    }
}

// Non-maximum suppression implementation
void nonMaxSuppression(uint8_t* gradientMagnitude, uint8_t* gradientDirection, uint8_t* nms) {
    for (int y = 1; y < HEIGHT - 1; y++) {
        for (int x = 1; x < WIDTH - 1; x++) {
            int idx = y * WIDTH + x;
            float angle = gradientDirection[idx];
            float mag = gradientMagnitude[idx];
            
            if (angle < 0) angle += 180;
            
            float mag1, mag2;
            
            if ((angle >= 0 && angle < 22.5) || (angle >= 157.5 && angle <= 180)) {
                mag1 = gradientMagnitude[idx-1];
                mag2 = gradientMagnitude[idx+1];
            }
            else if (angle >= 22.5 && angle < 67.5) {
                mag1 = gradientMagnitude[(y-1)*WIDTH + (x+1)];
                mag2 = gradientMagnitude[(y+1)*WIDTH + (x-1)];
            }
            else if (angle >= 67.5 && angle < 112.5) {
                mag1 = gradientMagnitude[(y-1)*WIDTH + x];
                mag2 = gradientMagnitude[(y+1)*WIDTH + x];
            }
            else {
                mag1 = gradientMagnitude[(y-1)*WIDTH + (x-1)];
                mag2 = gradientMagnitude[(y+1)*WIDTH + (x+1)];
            }
            
            nms[idx] = (mag >= mag1 && mag >= mag2) ? mag : 0;
        }
    }
}

// 1D conv
void convolution_1D(uint8_t *arr, size_t arr_len, float *kernel, size_t kernel_len, size_t dilation, uint8_t *output) {
    int y = 0;
    for (int x = 1; x < WIDTH - 1; x++) {
        float sum = 0;
            for (int kx = -1; kx <= 1; kx++) {
                sum += arr[y*WIDTH + (x+kx)] * kernel[kx+1];
            }
        // }
        output[y*WIDTH + x] = (uint8_t)sum;
    }
}

// Gaussian Blur implementation
void gaussian_blur(uint8_t* src, uint8_t* dst) {
    uint32_t in_len[1] = {WIDTH};
    uint16_t in_dilation[1] = {1};

    uint8_t test_out1[WIDTH * HEIGHT];
    for (int i = 0; i < 8; i++) {
        test_out1[(HEIGHT - 2)*WIDTH + i] = 0;
        test_out1[(HEIGHT - 1)*WIDTH + i] = 0;
    }
    uint8_t test_out2[WIDTH * HEIGHT];
    for (int i = 0; i < 8; i++) {
        test_out2[0*WIDTH + i] = 0;
        test_out2[(HEIGHT - 1)*WIDTH + i] = 0;
    }
    uint8_t test_out3[WIDTH * HEIGHT];
    for (int i = 0; i < 8; i++) {
        test_out3[0*WIDTH + i] = 0;
        test_out3[1*WIDTH + i] = 0;
    }

    for (int y = 0; y < HEIGHT - 2; y++) {
        float in_kernel1[8] = {1/16.0, 2/16.0, 1/16.0, 0, 0, 0, 0, 0};
        convolution_1D(src + y*WIDTH, in_len[0], in_kernel1, 8, in_dilation[0], test_out1 + y*WIDTH);
    }

    for (int y = 1; y < HEIGHT - 1; y++) {
        float in_kernel2[8] = {2/16.0, 4/16.0, 2/16.0, 0, 0, 0, 0, 0};
        convolution_1D(src + y*WIDTH, in_len[0], in_kernel2, 8, in_dilation[0], test_out2 + y*WIDTH);
    }

    for (int y = 2; y < HEIGHT; y++) {
        float in_kernel3[8] = {1/16.0, 2/16.0, 1/16.0, 0, 0, 0, 0, 0};
        convolution_1D(src + y*WIDTH, in_len[0], in_kernel3, 8, in_dilation[0], test_out3 + y*WIDTH);
    }

    for (int y = 1; y < HEIGHT - 1; y++) {
        for (int x = 0; x < WIDTH; x++) {
            dst[y*WIDTH+x] = test_out1[(y-1)*WIDTH+x] + test_out2[y*WIDTH+x] + test_out3[(y+1)*WIDTH+x];
        }
    }
}

// Sobel operator implementation
void sobelOperator(uint8_t* src, uint8_t* gradientMagnitude, uint8_t* gradientDirection) {
    const int Gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    const int Gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    for (int y = 1; y < HEIGHT - 1; y++) {
        for (int x = 1; x < WIDTH - 1; x++) {
            float gx = 0, gy = 0;
            
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    int idx = (y + i) * WIDTH + (x + j);
                    gx += src[idx] * Gx[i+1][j+1];
                    gy += src[idx] * Gy[i+1][j+1];
                }
            }

            int idx = y * WIDTH + x;
            gradientMagnitude[idx] = (uint8_t)sqrt(gx*gx + gy*gy);
            gradientDirection[idx] = (uint8_t)(atan2(gy, gx) * 180.0 / PI);
        }
    }
}

// Hysteresis implementation
void hysteresis(uint8_t* src, uint8_t* dst) {
    memcpy(dst, src, WIDTH * HEIGHT);
    int changed;
    
    do {
        changed = 0;
        for (int y = 1; y < HEIGHT - 1; y++) {
            for (int x = 1; x < WIDTH - 1; x++) {
                int idx = y * WIDTH + x;
                if (dst[idx] == 128) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int neighborIdx = (y + dy) * WIDTH + (x + dx);
                            if (dst[neighborIdx] == 255) {
                                dst[idx] = 255;
                                changed = 1;
                                break;
                            }
                        }
                        if (dst[idx] == 255) break;
                    }
                    if (dst[idx] == 128) {
                        dst[idx] = 0;
                    }
                }
            }
        }
    } while (changed);
}

void subjectIdentification(unsigned char* src, int* position) {
    float xSums[HEIGHT] = {0};
    float xIndices[HEIGHT] = {0};
    float ySums[WIDTH] = {0};
    float yIndices[HEIGHT] = {0};

    for (int y = 1; y < HEIGHT - 1; y++) {
        for (int x = 1; x < WIDTH - 1; x++) {
            if (src[y * WIDTH + x] == 255) {
                xSums[y] += x;
                xIndices[y] += 1;
                ySums[x] += y;
                yIndices[x] += 1;
            }
        }
    }

    float xSum = 0;
    for (int i = 0; i < HEIGHT; i++) {
        if (xIndices[i] != 0) {
            xSum += xSums[i] / xIndices[i];
        }
    }

    float ySum = 0;
    for (int i = 0; i < WIDTH; i++) {
        if (yIndices[i] != 0) {
            ySum += ySums[i] / yIndices[i];
        }
    }

    position[0] = (int) (xSum / HEIGHT);
    position[1] = (int) (ySum / WIDTH);
}

/* USER CODE END PUC */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Configure the system clock */
  /* Configure the system clock */
  
  /* USER CODE BEGIN SysInit */ 
  UART_InitType UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART1, &UART_init_config);

  PWM_InitType PWM_init_config;
  PWM_init_config.pwmscale = 0;
  PWM_init_config.RESERVED = 0;
  PWM_init_config.pwmsticky = 0;
  PWM_init_config.pwmzerocmp = 0;
  PWM_init_config.pwmdeglitch = 0;
  PWM_init_config.RESERVED1 = 0;
  PWM_init_config.pwmenalways = 0;
  PWM_init_config.pwmenoneshot = 0;
  PWM_init_config.RESERVED2 = 0;
  PWM_init_config.pwmcmp0center = 0;
  PWM_init_config.pwmcmp1center = 0;
  PWM_init_config.pwmcmp2center = 0;
  PWM_init_config.pwmcmp3center = 0;
  PWM_init_config.RESERVED3 = 0;
  PWM_init_config.pwmcmp0gang = 0;
  PWM_init_config.pwmcmp1gang = 0;
  PWM_init_config.pwmcmp2gang = 0;
  PWM_init_config.pwmcmp3gang = 0;
  PWM_init_config.pwmcmp0ip = 0;
  PWM_init_config.pwmcmp1ip = 0;
  PWM_init_config.pwmcmp2ip = 0;
  PWM_init_config.pwmcmp3ip = 0;
  pwm_init(PWM0_BASE, &PWM_init_config);
  *((uint32_t*) (PWM0_BASE+0x08)) = 0;

  pwm_enable(PWM0_BASE);

  pwm_set_frequency(PWM0_BASE, 0, 400);
  pwm_set_duty_cycle(PWM0_BASE, 0, 1, 400, 0);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */  
  /* USER CODE BEGIN Init */
  app_init();
  /* USER CODE END Init */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // while (1) {
  //   app_main();
  //   return 0;
  // }

  uint8_t image_data[WIDTH * HEIGHT]; 
  
  uint8_t blurred[WIDTH * HEIGHT] = {0};
  uint8_t gradientMagnitude[WIDTH * HEIGHT] = {0};
  uint8_t gradientDirection[WIDTH * HEIGHT] = {0};
  uint8_t nms[WIDTH * HEIGHT] = {0};
  uint8_t threshold[WIDTH * HEIGHT] = {0};
  uint8_t output[WIDTH * HEIGHT] = {0};
  int position[2] = {0,0};
  int test = 0;

  while (1) {
    // capture_image(image_data);
    // gaussian_blur(image_data, blurred);
    // sobelOperator(blurred, gradientMagnitude, gradientDirection);
    // nonMaxSuppression(gradientMagnitude, gradientDirection, nms);
    // doubleThreshold_opt(nms, threshold, WIDTH * HEIGHT, 0.2, 0.1);  // High threshold = 0.2, Low threshold = 0.1
    // hysteresis(threshold, output);
    // subjectIdentification(output, position);
    if (test) {
        position[0] = 320;
        position[1] = 240;
        test = 0;
    } else{ 
        position[0] = 0;
        position[1] = 0;
        test = 1;
    }
    calculate_pwm_to_center(position[0], position[1]);
  }

//   for (size_t x = 0; x < sizeof(output) / sizeof(output[0]); x++) {
//     printf("%c", (unsigned char)output[x]); // Send exactly 1 byte
//   }

  return 0;
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