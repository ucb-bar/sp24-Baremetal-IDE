/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : test_sensor_fusion.c
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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include "main.h"
#include "chip_config.h"
#include "icm42688.h"
#include "uart.h"


// IMU CALIBRATION (MUST MATCH QUAD.C)
#define ACCEL_X_OFFSET 1.05f
#define ACCEL_Y_OFFSET -7.33f
#define ACCEL_Z_OFFSET 6.04f

#define ACCEL_X_SCALE  1.0012f
#define ACCEL_Y_SCALE  1.0015f
#define ACCEL_Z_SCALE  1.0016f

#define ACCEL_SCALE ((16.0f / 32768.0f) * 9.81f)
#define GYRO_SCALE  ((2000.0f / 32768.0f) * (3.14159f / 180.0f))

float invSqrt(float x) { return 1.0f / sqrtf(x); }

/* =========================================================================
 * MADGWICK FILTER (Simplified for Test)
 * ========================================================================= */

typedef struct {
    float q0, q1, q2, q3;
    float beta;
} MadgwickFilter;

void madgwick_update(MadgwickFilter *f, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-f->q1 * gx - f->q2 * gy - f->q3 * gz);
    qDot2 = 0.5f * (f->q0 * gx + f->q2 * gz - f->q3 * gy);
    qDot3 = 0.5f * (f->q0 * gy - f->q1 * gz + f->q3 * gx);
    qDot4 = 0.5f * (f->q0 * gz + f->q1 * gy - f->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * f->q0; _2q1 = 2.0f * f->q1; _2q2 = 2.0f * f->q2; _2q3 = 2.0f * f->q3;
        _4q0 = 4.0f * f->q0; _4q1 = 4.0f * f->q1; _4q2 = 4.0f * f->q2;
        _8q1 = 8.0f * f->q1; _8q2 = 8.0f * f->q2;
        q0q0 = f->q0 * f->q0; q1q1 = f->q1 * f->q1; q2q2 = f->q2 * f->q2; q3q3 = f->q3 * f->q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * f->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * f->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * f->q3 - _2q1 * ax + 4.0f * q2q2 * f->q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= f->beta * s0; qDot2 -= f->beta * s1; qDot3 -= f->beta * s2; qDot4 -= f->beta * s3;
    }

    // Integrate to yield quaternion
    f->q0 += qDot1 * dt; f->q1 += qDot2 * dt; f->q2 += qDot3 * dt; f->q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= recipNorm; f->q1 *= recipNorm; f->q2 *= recipNorm; f->q3 *= recipNorm;
}

/* =========================================================================
 * APP MAIN
 * ========================================================================= */

static ICM42688_Data imu_data;
static MadgwickFilter filter = {1.0f, 0.0f, 0.0f, 0.0f, 0.1f}; // Beta 0.1

void app_init() {
    UART_InitType UART_conf = {115200, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART0, &UART_conf);

    I2C_InitType i2c_conf;
    i2c_conf.clock = 400000;
    i2c_init(I2C0, &i2c_conf);

    printf("\n\n=== ORIENTATION TEST ===\n");
    if (icm42688_init(I2C0, CLINT) != 0) {
        printf("IMU Init Failed!\n");
        while(1);
    }
    printf("IMU Ready. Keep level for calibration check...\n");
}

void app_main() {
    uint64_t last_time = get_time_us();
    int print_div = 0;

    while (1) {
        // 1. Time delta
        uint64_t now = get_time_us();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;
        //printf("IMU Ready. Keep level for calibration check...\n");

        // 2. Read IMU
        icm42688_read_all(I2C0, &imu_data, CLINT);
        
        // APPLY CALIBRATION
        float ax = (imu_data.accel_x - ACCEL_X_OFFSET) * ACCEL_X_SCALE * (9.81f / 2048.0f);
        float ay = (imu_data.accel_y - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE * (9.81f / 2048.0f);
        float az = (imu_data.accel_z - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE * (9.81f / 2048.0f);
        float gx = imu_data.gyro_x * GYRO_SCALE;
        float gy = imu_data.gyro_y * GYRO_SCALE;
        float gz = imu_data.gyro_z * GYRO_SCALE;

        // 3. Update Filter
        madgwick_update(&filter, gx, gy, gz, ax, ay, az, dt);

        // 4. Convert Quaternion to Euler (Degrees)
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (filter.q0 * filter.q1 + filter.q2 * filter.q3);
        float cosr_cosp = 1.0f - 2.0f * (filter.q1 * filter.q1 + filter.q2 * filter.q2);
        float roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / 3.14159f);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (filter.q0 * filter.q2 - filter.q3 * filter.q1);
        float pitch;
        if (fabs(sinp) >= 1) pitch = copysignf(90.0f, sinp); // use 90 degrees if out of range
        else pitch = asinf(sinp) * (180.0f / 3.14159f);

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2);
        float cosy_cosp = 1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3);
        float yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.14159f);

        // 5. Print Output (Every 100ms)
        if (print_div++ > 10) { 
            printf("R: %6.3f  P: %6.3f  Y: %6.3f  (dt: %.6f)\r\n", roll, pitch, yaw, dt);
            print_div = 0;
        }
        
        // Small delay to prevent running too fast for UART
        for(volatile int k=0; k<20000; k++); 
    }
}

int main(int argc, char **argv) {
    app_init();
    app_main();
    return 0;
}
void __attribute__((weak, noreturn)) __main(void) { while (1) asm("wfi"); }