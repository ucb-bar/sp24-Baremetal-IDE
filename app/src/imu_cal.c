#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include "main.h"
#include "chip_config.h"
#include "icm42688.h"
#include "uart.h"

#define SAMPLES 1000.0f
#define G_REF   2048.0f // Assuming +/- 16g scale -> 32768/16 = 2048 LSB/g

// Struct to hold raw averages
typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

static ICM42688_Data raw_data;
static Vector3f offset = {0,0,0};
static Vector3f scale  = {1,1,1};

/* =========================================================================
 * HELPER FUNCTIONS
 * ========================================================================= */

void get_average_reading(Vector3f *avg) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    
    printf("   Sampling...");
    for (int i = 0; i < SAMPLES; i++) {
        if (icm42688_read_all(I2C0, &raw_data, CLINT) != 0) {
            printf("ERROR: IMU Read Failed.\n");
            while(1);
        }
        //printf("SUCCESS: IMU Valid. Accel X: %d, Accel Z: %d\n", raw_data.accel_x, raw_data.accel_z);
        sum_x += raw_data.accel_x;
        sum_y += raw_data.accel_y;
        sum_z += raw_data.accel_z;
        //printf("SUCCESS: IMU Valid. Sum X: %d, Sum Z: %d\n", sum_x, sum_z);
        // Small delay to get diverse samples
        for(volatile int k=0; k<10000; k++); 
    }

    //printf(" Done. (%d, %d, %d)\n", sum_x, sum_y, sum_z);
    
    avg->x = (float)sum_x / SAMPLES;
    avg->y = (float)sum_y / SAMPLES;
    avg->z = (float)sum_z / SAMPLES;
    printf(" Done. (%f, %f, %f)\n", avg->x, avg->y, avg->z);
}

/* =========================================================================
 * MAIN CALIBRATION LOGIC
 * ========================================================================= */

void app_init() {
    UART_InitType UART_conf = {115200, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART0, &UART_conf);

    I2C_InitType i2c_conf;
    i2c_conf.clock = 400000;
    i2c_init(I2C0, &i2c_conf);

    printf("\n\n=== IMU 6-POINT CALIBRATION ===\n");
    if (icm42688_init(I2C0, CLINT) != 0) {
        printf("IMU Init Failed!\n");
        while(1);
    }
    printf("IMU Connected.\n");
}

void app_main() {
    Vector3f z_up, z_down, x_up, x_down, y_up, y_down;

    printf("\nInstructions:\n");
    printf("Place the drone in the requested orientation and press 'c' to capture.\n");
    printf("Ensure it is perfectly still during sampling.\n\n");

    // 1. Z Axis Calibration
    printf("[1/6] LEVEL (Z+ Up, Flat on table). Press 'c'.\n");
    char a[1];
    uart_receive(UART0, a, 1, 100000); // Blocking wait
    if (a[0] != 'c') return;
    get_average_reading(&z_up);

    printf("[2/6] UPSIDE DOWN (Z- Up). Press 'c'.\n");
    char b[1];
    uart_receive(UART0, b, 1, 100000); // Blocking wait
    if (b[0] != 'c') return;
    get_average_reading(&z_down);

    // 2. X Axis Calibration
    printf("[3/6] NOSE UP (X+ Up). Press 'c'.\n");
    char c[1];
    uart_receive(UART0, c, 1, 100000); // Blocking wait
    if (c[0] != 'c') return;
    get_average_reading(&x_up);

    printf("[4/6] NOSE DOWN (X- Up). Press 'c'.\n");
    char d[1];
    uart_receive(UART0, d, 1, 100000); // Blocking wait
    if (d[0] != 'c') return;
    get_average_reading(&x_down);

    // 3. Y Axis Calibration
    printf("[5/6] LEFT SIDE UP (Y+ Up). Press 'c'.\n");
    char e[1];
    uart_receive(UART0, e, 1, 100000); // Blocking wait
    if (e[0] != 'c') return;
    get_average_reading(&y_up);

    printf("[6/6] RIGHT SIDE UP (Y- Up). Press 'c'.\n");
    char f[1];
    uart_receive(UART0, f, 1, 100000); // Blocking wait
    if (f[0] != 'c') return;
    get_average_reading(&y_down);

    // --- CALCULATIONS ---
    // Offset = (Max + Min) / 2
    // Scale  = G_REF / ((Max - Min) / 2)
    
    // X Axis
    offset.x = (x_up.x + x_down.x) / 2.0f;
    scale.x  = G_REF / ((x_up.x - x_down.x) / 2.0f);

    // Y Axis
    offset.y = (y_up.y + y_down.y) / 2.0f;
    scale.y  = G_REF / ((y_up.y - y_down.y) / 2.0f);

    // Z Axis
    offset.z = (z_up.z + z_down.z) / 2.0f;
    scale.z  = G_REF / ((z_up.z - z_down.z) / 2.0f);

    printf("\n\n=== CALIBRATION RESULTS ===\n");
    printf("Copy these values into your quadcopter_main.c:\n\n");
    
    printf("// IMU OFFSETS\n");
    printf("#define ACCEL_X_OFFSET %.2ff\n", offset.x);
    printf("#define ACCEL_Y_OFFSET %.2ff\n", offset.y);
    printf("#define ACCEL_Z_OFFSET %.2ff\n", offset.z);
    
    printf("\n// IMU SCALING\n");
    printf("#define ACCEL_X_SCALE  %.4ff\n", scale.x);
    printf("#define ACCEL_Y_SCALE  %.4ff\n", scale.y);
    printf("#define ACCEL_Z_SCALE  %.4ff\n", scale.z);

    printf("\n// Corrected Read Function:\n");
    printf("float cal_ax = (raw_ax - ACCEL_X_OFFSET) * ACCEL_X_SCALE * (9.81f / 2048.0f);\n");
    printf("float cal_ay = (raw_ay - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE * (9.81f / 2048.0f);\n");
    printf("float cal_az = (raw_az - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE * (9.81f / 2048.0f);\n");

    while(1);
}

int main(int argc, char **argv) {
    app_init();
    app_main();
    return 0;
}

void __attribute__((weak, noreturn)) __main(void) {
  uint64_t mhartid = READ_CSR("mhartid");
  while (1) {
    asm volatile("wfi");
  }
}
