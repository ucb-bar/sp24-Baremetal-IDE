/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : quad.c
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
#include <stdarg.h> 
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h> 
#include <unistd.h>
#include "main.h"
#include "chip_config.h"
#include "icm42688.h" 
#include "uart.h" 

/* =========================================================================
 * CONSTANTS & CONFIG
 * ========================================================================= */

// Physics Constants
const float mass = 35e-3f;
const float gravity = 9.81f;
const float inertia_xx = 16e-6f;
const float inertia_yy = 16e-6f;
const float inertia_zz = 29e-6f;
const float rho = 0.15f;
const float l = 33e-3f;
const float k = 0.01f;

// PID / Control Constants
const float tau_roll = 0.10f;
const float tau_pitch = 0.10f;
const float tau_yaw = 0.25f;
const float tau_rollRate = 0.025f;
const float tau_pitchRate = 0.025f;
const float tau_yawRate = 0.05f;
const float timeConst_horizVel = 0.5f;
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

// Ardupilot Quad-X Motor Mapping
// Ch0: Motor 1 (Front Right, CCW)
// Ch1: Motor 2 (Rear Left, CCW)
// Ch2: Motor 3 (Front Left, CW)
// Ch3: Motor 4 (Rear Right, CW)
#define MOTOR1_PWM_CH 0
#define MOTOR2_PWM_CH 1
#define MOTOR3_PWM_CH 2
#define MOTOR4_PWM_CH 3

#define ACCEL_SCALE ((16.0f / 32768.0f) * 9.81f)
#define GYRO_SCALE  ((2000.0f / 32768.0f) * (3.14159f / 180.0f))

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* =========================================================================
 * DATA STRUCTURES
 * ========================================================================= */

typedef struct {
    float q0, q1, q2, q3;
    float beta;
    float sample_freq;
} MadgwickFilter;

typedef struct {
    float estRoll, estPitch, estYaw, estHeight;
    float estVel_1, estVel_2, estVel_3;
    float global_pos_x, global_pos_y;
    int16_t global_pos_z_mm; 

    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    
    int16_t motion_count_x, motion_count_y;
} DroneState;

/* =========================================================================
 * GLOBAL VARIABLES
 * ========================================================================= */

static DroneState state;
static MadgwickFilter filter;
static ICM42688_Data imu_data;

static float motor_cmds[4] = {0};
static float lastHeightMeas_meas = 0;
static uint64_t last_loop_time_us = 0;

static int error_flag = 0;
static int init_flag = 0;
static int descend_flag = 0;
static int done_flag = 0;

// MIXING MATRIX (Ardupilot Quad-X)
// Rows: M1(FR), M2(RL), M3(FL), M4(RR)
// Cols: Thrust, Roll, Pitch, Yaw
// 
// Logic Check:
// Roll Right (+): Left (M2, M3) Up, Right (M1, M4) Down
// Pitch Up (+): Rear (M2, M4) Up, Front (M1, M3) Down
// Yaw Right (+): CCW Motors (M1, M2) Up, CW Motors (M3, M4) Down
const float M[4][4] = {
    {0.25f, -0.25f/l, -0.25f/l,  0.25f/k}, // M1: FR (Right, Front, CCW) -> R-, P-, Y+
    {0.25f,  0.25f/l,  0.25f/l,  0.25f/k}, // M2: RL (Left, Rear, CCW)  -> R+, P+, Y+
    {0.25f,  0.25f/l, -0.25f/l, -0.25f/k}, // M3: FL (Left, Front, CW)  -> R+, P-, Y-
    {0.25f, -0.25f/l,  0.25f/l, -0.25f/k}  // M4: RR (Right, Rear, CW)  -> R-, P+, Y-
};

const float J[3][3] = {
    {16e-6f, 0, 0}, {0, 16e-6f, 0}, {0, 0, 29e-6f}
};

/* =========================================================================
 * HELPER FUNCTIONS
 * ========================================================================= */

float invSqrt(float x) { return 1.0f / sqrtf(x); }

/* =========================================================================
 * SENSOR & FILTER
 * ========================================================================= */

void sensor_init_all() {
    printf("Initializing Sensors...\n");
    if (icm42688_init(I2C0, CLINT) == 0) {
        printf("ICM-42688 Init Success!\n");
    } else {
        printf("ICM-42688 Init Failed.\n");
        error_flag = 1; 
    }

    if (vl53l1x_init(I2C0, CLINT) == 0) {
        printf("VL53L1X Init Success!\n");

        // Example: One-time Offset Calibration 
        // Only run this if you know the drone is exactly 140mm from ground!
        /*
        int16_t offset;
        printf("Calibrating ToF Offset (Target=140mm)...\n");
        vl53l1x_calibrate_offset(I2C0, CLINT, 140, &offset);
        printf("New Offset: %d\n", offset);
        */
       
    } else {
        printf("VL53L1X Init Failed.\n");
        // Warning only, flight can continue without altitude hold
    }
}

void read_imu_burst(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    if (icm42688_read_all(I2C0, &imu_data, CLINT) == 0) {
        *ax = imu_data.accel_x * ACCEL_SCALE;
        *ay = imu_data.accel_y * ACCEL_SCALE;
        *az = imu_data.accel_z * ACCEL_SCALE;
        *gx = imu_data.gyro_x * GYRO_SCALE;
        *gy = imu_data.gyro_y * GYRO_SCALE;
        *gz = imu_data.gyro_z * GYRO_SCALE;
    }
}

int16_t read_tof_distance() {
    int16_t dist = vl53l1x_read_distance(I2C0, CLINT);
    if (dist < 0) return 0; // Return 0 on error
    return dist;
}

void madgwick_init(MadgwickFilter *f, float sample_freq) {
    f->beta = 0.1f; f->q0 = 1.0f; f->q1 = 0.0f; f->q2 = 0.0f; f->q3 = 0.0f;
    f->sample_freq = sample_freq;
}

void madgwick_update(MadgwickFilter *f, float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-f->q1 * gx - f->q2 * gy - f->q3 * gz);
    qDot2 = 0.5f * (f->q0 * gx + f->q2 * gz - f->q3 * gy);
    qDot3 = 0.5f * (f->q0 * gy - f->q1 * gz + f->q3 * gx);
    qDot4 = 0.5f * (f->q0 * gz + f->q1 * gy - f->q2 * gx);

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f * f->q0; _2q1 = 2.0f * f->q1; _2q2 = 2.0f * f->q2; _2q3 = 2.0f * f->q3;
        _4q0 = 4.0f * f->q0; _4q1 = 4.0f * f->q1; _4q2 = 4.0f * f->q2;
        _8q1 = 8.0f * f->q1; _8q2 = 8.0f * f->q2;
        q0q0 = f->q0 * f->q0; q1q1 = f->q1 * f->q1; q2q2 = f->q2 * f->q2; q3q3 = f->q3 * f->q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * f->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * f->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * f->q3 - _2q1 * ax + 4.0f * q2q2 * f->q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= f->beta * s0; qDot2 -= f->beta * s1; qDot3 -= f->beta * s2; qDot4 -= f->beta * s3;
    }

    float dt = 1.0f / f->sample_freq; 
    f->q0 += qDot1 * dt; f->q1 += qDot2 * dt; f->q2 += qDot3 * dt; f->q3 += qDot4 * dt;
    recipNorm = invSqrt(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= recipNorm; f->q1 *= recipNorm; f->q2 *= recipNorm; f->q3 *= recipNorm;
}

/* =========================================================================
 * MOTOR CONTROL
 * ========================================================================= */

int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
    float a = -100.849f; float b = 0.1261846f;
    return (int)(a + b * desiredSpeed_rad_per_sec);
}

float speedFromForce(float desiredForce_N) {
    if (desiredForce_N <= 0) return 0.0f;
    return sqrtf(desiredForce_N / 2.0e-08f);
}

float forceToVoltage(float forceNewtons) {
    float cmd = pwmCommandFromSpeed(speedFromForce(forceNewtons)) / 255.0f;
    if (cmd > 1.0f) cmd = 1.0f; else if (cmd < 0.0f) cmd = 0.0f;
    return cmd;
}

void set_motors(float m1, float m2, float m3, float m4) {
    pwm_set_duty_cycle(PWM0_BASE, MOTOR1_PWM_CH, (uint32_t)(m1 * 100.0f), 0);
    pwm_set_duty_cycle(PWM0_BASE, MOTOR2_PWM_CH, (uint32_t)(m2 * 100.0f), 0);
    pwm_set_duty_cycle(PWM0_BASE, MOTOR3_PWM_CH, (uint32_t)(m3 * 100.0f), 0);
    pwm_set_duty_cycle(PWM0_BASE, MOTOR4_PWM_CH, (uint32_t)(m4 * 100.0f), 0);
}

/* =========================================================================
 * CONTROL LOOP
 * ========================================================================= */

void control_step(float dt) {
    read_imu_burst(&state.accelX, &state.accelY, &state.accelZ, 
                   &state.gyroX, &state.gyroY, &state.gyroZ);

    // Note: In a real loop, you might only read this every 50ms (20Hz) 
    // to save I2C bus time, as ToF is slower than IMU.
    int16_t alt_mm = read_tof_distance();
    state.estHeight = alt_mm / 1000.0f; // Convert mm to meters
    
    float accRoll  = state.accelY / gravity;
    float accPitch = -state.accelX / gravity;
    
    state.estRoll  = (1.0f - rho) * (state.estRoll + dt * state.gyroX)  + rho * accRoll;
    state.estPitch = (1.0f - rho) * (state.estPitch + dt * state.gyroY) + rho * accPitch;
    state.estYaw   = state.estYaw + dt * state.gyroZ;

    if (state.accelZ < -40.0f || state.accelZ > 40.0f) { error_flag = 1; }

    float desHeight = 0.75f;
    uint32_t now_ms = get_time_us() / 1000;
    if (now_ms > 3000) init_flag = 1;
    if (now_ms > 6000) descend_flag = 1;
    if (now_ms > 10000) done_flag = 1;

    if (descend_flag) {
        desHeight -= 0.3f * dt;
        if (desHeight < -0.2f) desHeight = -0.2f;
    } else {
        desHeight -= 0.05f * dt;
    }

    float desAcc1 = -(1.0f / timeConst_horizVel) * state.estVel_1;
    float desAcc2 = -(1.0f / timeConst_horizVel) * state.estVel_2;
    float desRoll = -desAcc2 / gravity;
    float desPitch = desAcc1 / gravity;
    float desYaw = state.estYaw;

    float rollRate_tgt = (-1.0f / tau_roll) * (state.estRoll - desRoll);
    float pitchRate_tgt = (-1.0f / tau_pitch) * (state.estPitch - desPitch);
    float yawRate_tgt = (-1.0f / tau_yaw) * (state.estYaw - desYaw);

    float rollRate_cmd = (-1.0f / tau_rollRate) * (state.gyroX - rollRate_tgt);
    float pitchRate_cmd = (-1.0f / tau_pitchRate) * (state.gyroY - pitchRate_tgt);
    float yawRate_cmd = (-1.0f / tau_yawRate) * (state.gyroZ - yawRate_tgt);

    const float desAcc3 = -2.0f * dampingRatio_height * natFreq_height * state.estVel_3 
                          - natFreq_height * natFreq_height * (state.estHeight - desHeight);
    float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(state.estRoll) * cosf(state.estPitch));
    
    float u[4] = {desNormalizedAcceleration * mass, 0, 0, 0};
    u[1] = rollRate_cmd * J[0][0];
    u[2] = pitchRate_cmd * J[1][1];
    u[3] = yawRate_cmd * J[2][2];

    float ctrl[4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ctrl[i] += M[i][j] * u[j];
        }
    }

    motor_cmds[0] = forceToVoltage(0.9f * ctrl[1]);
    motor_cmds[1] = forceToVoltage(0.9f * ctrl[2]);
    motor_cmds[2] = forceToVoltage(0.9f * ctrl[3] * 0.87f);
    motor_cmds[3] = forceToVoltage(0.9f * ctrl[0] * 0.87f);

    if (init_flag && !done_flag && !error_flag) {
        set_motors(motor_cmds[0], motor_cmds[1], motor_cmds[2], motor_cmds[3]);
    } else {
        set_motors(0, 0, 0, 0);
    }
}

/* =========================================================================
 * INIT & MAIN
 * ========================================================================= */

void app_init() {
    pwm_enable(PWM0_BASE);
    pwm_set_frequency(PWM0_BASE, 0, 25000);

    UART_InitType UART0_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_2};
    uart_init(UART0, &UART0_init_config);

    UART_InitType UART1_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART1, &UART1_init_config);

    I2C_InitType i2c_conf;
    i2c_conf.clock = 400000;
    i2c_init(I2C0, &i2c_conf);
    
    sensor_init_all();
    madgwick_init(&filter, 100.0f);

    set_motors(0.1f, 0, 0, 0); msleep(250);
    set_motors(0, 0.1f, 0, 0); msleep(250);
    set_motors(0, 0, 0.1f, 0); msleep(250);
    set_motors(0, 0, 0, 0.1f); msleep(250);
    set_motors(0, 0, 0, 0);
    
    printf("Quad init done (Ardupilot Layout).\n");
}

void app_main() {
    uint64_t last_control_us = get_time_us();
    uint64_t last_print_us = get_time_us();
    uint64_t now;

    while(1) {
        now = get_time_us();
        if ((now - last_control_us) >= 10000) {
            float dt = (float)(now - last_control_us) / 1000000.0f;
            last_control_us = now;
            control_step(dt);
            
             if (error_flag) gpio_write_pin(GPIOC, GPIO_PIN_0, 1); 
             else if (init_flag) gpio_write_pin(GPIOC, GPIO_PIN_1, 1);
        }

        if ((now - last_print_us) >= 1000000) {
            last_print_us = now;
            printf("R:%.2f P:%.2f H:%.2f | M:%.2f %.2f %.2f %.2f\n", 
                state.estRoll, state.estPitch, state.estHeight,
                motor_cmds[0], motor_cmds[1], motor_cmds[2], motor_cmds[3]);
        }
    }
}

int main(int argc, char **argv) {
    app_init();
    app_main();
    return 0;
}

void handle_sigint(int sig) {
    printf("\nCaught signal %d, exiting...\n", sig);
}

void __attribute__((weak, noreturn)) __main(void) {
  while (1) {
    asm volatile("wfi");
  }
}