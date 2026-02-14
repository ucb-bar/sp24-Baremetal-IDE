/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : quad_pid_madgwick_zkalman.c
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
#include "vl53l1x.h" 
#include "uart.h" 

/* =========================================================================
 * CONSTANTS & CONFIG
 * ========================================================================= */

// Physics Constants
const float mass = 255e-3f;
const float gravity = 9.81f;
const float inertia_xx = 37e-5f;
const float inertia_yy = 23.9e-5f;
const float inertia_zz = 49.7e-5f;
const float rho = 0.15f;
const float l = 88.9e-3f;
const float k = 0.1f;


// --- PID GAINS ---
// P = Strength of correction
// I = Memory of error (Fixes drift/imbalance)
// D = Damping (Stops oscillation)

// RATE LOOPS (Inner Loop - Fast)
// High P makes it feel "Locked in". High D stops bounce.
const float kp_roll  = 0.13f;  const float ki_roll  = 0.005f; const float kd_roll  = 0.00f;
const float kp_pitch = 0.13f;  const float ki_pitch = 0.005f; const float kd_pitch = 0.00f;
const float kp_yaw   = 0.07f;  const float ki_yaw   = 0.005f; const float kd_yaw   = 0.00f;

// ANGLE LOOPS (Outer Loop - Stabilization)
// Converts Angle Error -> Target Rate
const float kp_angle = 1.0f; // If 10 deg error, command 60 deg/sec correction

const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

// IMU OFFSETS
#define ACCEL_X_OFFSET 1.05f
#define ACCEL_Y_OFFSET -7.33f
#define ACCEL_Z_OFFSET 6.04f

// IMU SCALING
#define ACCEL_X_SCALE  1.0012f
#define ACCEL_Y_SCALE  1.0015f
#define ACCEL_Z_SCALE  1.0016f

// INERTIAL FILTER GAINS (For Altitude Z)
// Defines how strongly we correct Accel drift using ToF data
const float z_k_h = 2.0f;   // Position correction gain
const float z_k_v = 5.0f;   // Velocity correction gain

// Ardupilot Quad-X Motor Mapping
// Ch0: Motor 1 (Front Right, CCW)
// Ch1: Motor 2 (Rear Left, CCW)
// Ch2: Motor 3 (Front Left, CW)
// Ch3: Motor 4 (Rear Right, CW)
#define MOTOR1_PWM_CH 0
#define MOTOR2_PWM_CH 1
#define MOTOR3_PWM_CH 2
#define MOTOR4_PWM_CH 3

// MOTOR MAPPING (The Fix)
// Define the duty cycle (0.0 - 1.0) where each motor *actually* starts spinning.
// You found: Two at 58% (0.58), Two at 50% (0.50). 
const float MOTOR_START_DUTY[4] = {0.54f, 0.54f, 0.54f, 0.54f}; // Adjust order to match M1, M2, M3, M4
const float MOTOR_MAX_DUTY = 0.95f;

#define MOT_FREQ_HZ 381

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
} MadgwickFilter;

typedef struct {
    float integral_err;
    float last_err;
} PID_State;

typedef struct {
    float estRoll, estPitch, estYaw, estHeight;
    float estVel_3;
    float accelX, accelY, accelZ; // Body Frame
    float gyroX, gyroY, gyroZ;
    float accelZ_earth; // Earth Frame (Vertical Acceleration)
    float global_pos_x, global_pos_y; // For future use with optical flow
    int16_t global_pos_z_mm; // For future use with optical flow

    int16_t motion_count_x, motion_count_y; // For future use with optical flow
} DroneState;

/* =========================================================================
 * GLOBAL VARIABLES
 * ========================================================================= */

static DroneState state;
static MadgwickFilter filter;
static ICM42688_Data imu_data;
static PID_State pid_roll, pid_pitch, pid_yaw;

static float motor_cmds[4] = {0};
static float lastHeightMeas_meas = 0;
static uint64_t last_loop_time_us = 0;

static int error_flag = 0;
static int init_flag = 0;
static int descend_flag = 0;
static int done_flag = 0;

float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

float accel_bias_x = 0.0f;
float accel_bias_y = 0.0f;
float accel_bias_z = 0.0f;

static float last_tof_h = 0.0f;
static int tof_valid = 0;

static int lpf_inited = 0;
static float ax_f, ay_f, az_f;

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

// MOMENT OF INERTIA (J)
// Calculated based on 260g total mass, 7.15g motors, and measured hub dimensions.
const float J[3][3] = {
    {0.000370f, 0, 0}, // Ixx (Roll Inertia)
    {0, 0.000239f, 0}, // Iyy (Pitch Inertia - Lower because body is narrower in Width)
    {0, 0, 0.000497f}  // Izz (Yaw Inertia)
};

/* =========================================================================
 * HELPER FUNCTIONS
 * ========================================================================= */

float invSqrt(float x) { return 1.0f / sqrtf(x); }

// Normalize angle to -PI to +PI
float normalize_angle(float angle) {
    while (angle > (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/* =========================================================================
 * SENSOR & FILTER
 * ========================================================================= */

void calibrate_gyro_bias() {
    printf("Calibrating Gyro... KEEP STILL!\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        icm42688_read_all(I2C0, &imu_data, CLINT);
        sum_x += imu_data.gyro_x;
        sum_y += imu_data.gyro_y;
        sum_z += imu_data.gyro_z;
        for(volatile int k=0; k<50000; k++); // Small delay
    }
    
    gyro_bias_x = sum_x / samples;
    gyro_bias_y = sum_y / samples;
    gyro_bias_z = sum_z / samples;
    
    printf("Gyro Bias: X:%.4f Y:%.4f Z:%.4f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

void calibrate_accel_bias() {
    printf("Calibrating Accelerometer... KEEP STILL!\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        icm42688_read_all(I2C0, &imu_data, CLINT);
        sum_x += imu_data.accel_x;
        sum_y += imu_data.accel_y;
        sum_z += imu_data.accel_z;
        for(volatile int k=0; k<50000; k++); // Small delay
    }
    
    accel_bias_x = sum_x / samples;
    accel_bias_y = sum_y / samples;
    accel_bias_z = (sum_z / samples) - (gravity/ACCEL_SCALE); // Remove gravity component from Z bias
    
    printf("Accelerometer Bias: X:%.4f Y:%.4f Z:%.4f\n", accel_bias_x, accel_bias_y, accel_bias_z);
}

void sensor_init_all() {
    printf("Initializing Sensors...\n");
    if (icm42688_init(I2C0, CLINT) == 0) {
        printf("ICM-42688 Init Success!\n");
    } else {
        printf("ICM-42688 Init Failed.\n");
        error_flag = 1; 
    }

    // If yaw drift is significant, run gyro calibration
    calibrate_gyro_bias();

    //1-D Accel bias calculation
    calibrate_accel_bias();

    if (vl53l1x_init(I2C1, CLINT) == 0) {
        printf("VL53L1X Init Success!\n");

        // Example: One-time Offset Calibration 
        // Only run this if you know the drone is exactly 140mm from ground!
        /*
        int16_t offset;
        printf("Calibrating ToF Offset (Target=140mm)...\n");
        vl53l1x_calibrate_offset(I2C1, CLINT, 140, &offset);
        printf("New Offset: %d\n", offset);
        */
       
    } else {
        printf("VL53L1X Init Failed.\n");
        // Warning only, flight can continue without altitude hold
    }
}

void read_imu_burst(float dt) {
    if (icm42688_read_all(I2C0, &imu_data, CLINT) == 0) {
        state.accelX = (imu_data.accel_x - ACCEL_X_OFFSET - accel_bias_x) * ACCEL_X_SCALE * (9.81f / 2048.0f);
        state.accelY = (imu_data.accel_y - ACCEL_Y_OFFSET - accel_bias_y) * ACCEL_Y_SCALE * (9.81f / 2048.0f);
        state.accelZ = (imu_data.accel_z - ACCEL_Z_OFFSET - accel_bias_z) * ACCEL_Z_SCALE * (9.81f / 2048.0f);
        //printf("Accelerations (m/s^2): X: %.3f Y: %.3f Z: %.3f\n", state.accelX, state.accelY, state.accelZ);
        //state.gyroX  = imu_data.gyro_x * GYRO_SCALE;
        //state.gyroY  = imu_data.gyro_y * GYRO_SCALE;
        //state.gyroZ  = imu_data.gyro_z * GYRO_SCALE;

        state.gyroX = (imu_data.gyro_x - gyro_bias_x) * GYRO_SCALE;
        state.gyroY = (imu_data.gyro_y - gyro_bias_y) * GYRO_SCALE;
        state.gyroZ = (imu_data.gyro_z - gyro_bias_z) * GYRO_SCALE;
        //printf("Gyro (rad/s): Y: %.3f\n", state.gyroY);


        
        // --- INERTIAL Z ESTIMATION (Simplified Kalman) ---
        // 1. Rotate Body Accel Z to Earth Frame
        // Approximation for small angles: Az_earth = Az_body - g (roughly)
        // Better: Use Quaternion to rotate [0, 0, Az] to earth frame.
        // Simplified Logic: 
        // Earth Accel Z = (AccelZ * cos(Roll) * cos(Pitch)) - Gravity
        // This removes the "tilt component" from the accelerometer.
        //float tilt_correction = cosf(state.estRoll) * cosf(state.estPitch);
        //state.accelZ_earth = (state.accelZ * tilt_correction) - gravity; 
        
        // 2. Predict (Inertial Integration)
        //state.estHeight += state.estVel_3 * dt;
        //state.estVel_3  += state.accelZ_earth * dt;
    }
}

void update_tof_fusion(float dt) {
    /*int16_t alt_mm = vl53l1x_read_distance(I2C1, CLINT);
    if (alt_mm >= 0) {
        float tof_h = alt_mm / 1000.0f;
        
        // 3. Correct (Fuse ToF with Inertial Estimate)
        // Error = Measured - Predicted
        float pos_err = tof_h - state.estHeight;
        
        // Apply Corrections
        state.estHeight += z_k_h * pos_err * dt;
        state.estVel_3  += z_k_v * pos_err * dt;
    }*/

    int16_t alt_mm = vl53l1x_read_distance(I2C1, CLINT);
    if (alt_mm < 0) return;

    float raw_h = alt_mm / 1000.0f;

    // Tilt compensation: convert slant range to vertical height
    float cos_tilt = cosf(state.estRoll) * cosf(state.estPitch);

    // Guard against crazy tilt / division noise
    if (cos_tilt < 0.5f) {   // ~60 deg
        // Too tilted: ToF vertical estimate unreliable; either ignore or heavily filter
        return;
    }

    float h = raw_h * cos_tilt;  // vertical height estimate

    // 1) Low-pass height (cut jitter)
    const float alpha_h = 0.2f; // 0..1 (higher = less filtering)
    if (!tof_valid) {
        state.estHeight = h;
        last_tof_h = h;
        tof_valid = 1;
        state.estVel_3 = 0.0f;
        return;
    }
    state.estHeight = (1.0f - alpha_h) * state.estHeight + alpha_h * h;

    // 2) Velocity from ToF derivative + low-pass
    float v_meas = (h - last_tof_h) / dt;
    last_tof_h = h;

    const float alpha_v = 0.15f;
    state.estVel_3 = (1.0f - alpha_v) * state.estVel_3 + alpha_v * v_meas;
}

void madgwick_init(MadgwickFilter *f) {
    f->beta = 0.02f; f->q0 = 1.0f; f->q1 = 0.0f; f->q2 = 0.0f; f->q3 = 0.0f;
}

// Safe inv sqrt (prevents NaN/Inf when x -> 0)
static inline float invSqrt_safe(float x) {
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

// Canonical Madgwick IMU-only update (gyro + accel, no mag).
// gx,gy,gz must be in rad/s. ax,ay,az can be in m/s^2 or g's (only direction matters).
void madgwick_update_imu(MadgwickFilter *f,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    // Short name local vars
    float q0 = f->q0;
    float q1 = f->q1;
    float q2 = f->q2;
    float q3 = f->q3;

    // Rate of change of quaternion from gyro (qDot = 0.5 * q ⊗ ω)
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accel measurement is valid
    // (avoids NaN in normalization)
    float a_norm = ax*ax + ay*ay + az*az;
    if (a_norm > 1e-12f) {
        recipNorm = invSqrt_safe(a_norm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        // Gradient descent corrective step (IMU-only reference)
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
           - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
           - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        // Normalize step magnitude (critical guard!)
        float s_norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
        if (s_norm > 1e-12f) {
            recipNorm = invSqrt_safe(s_norm);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot0 -= f->beta * s0;
            qDot1 -= f->beta * s1;
            qDot2 -= f->beta * s2;
            qDot3 -= f->beta * s3;
        }
        // else: skip accel correction this cycle (already aligned / step too small)
    }

    // Integrate to yield quaternion
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalize quaternion
    float q_norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (q_norm > 1e-12f) {
        recipNorm = invSqrt_safe(q_norm);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    } else {
        // Hard reset if something went catastrophically wrong
        q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    }

    // Write back
    f->q0 = q0;
    f->q1 = q1;
    f->q2 = q2;
    f->q3 = q3;
}

float pid_update(PID_State *pid, float error, float dt, float kp, float ki, float kd, int airborne) {
    // 1. Proportional
    float p_term = kp * error;
    
    // 2. Integral (Accumulate)
    // Clamp integral to prevent windup (max 20% authority)
    // ANTI-WINDUP: Only integrate if airborne!
    // This prevents the "Flip on Takeoff" due to ground windup.
    if (!airborne) {
        pid->integral_err = 0.0f;
    } else {
        pid->integral_err += error * dt;
        if (pid->integral_err > 2.0f) pid->integral_err = 2.0f;
        if (pid->integral_err < -2.0f) pid->integral_err = -2.0f;
    }
    float i_term = ki * pid->integral_err;

    // 3. Derivative (Change)
    float d_term = kd * (error - pid->last_err) / dt;
    pid->last_err = error;

    return p_term + i_term + d_term;
}

/* =========================================================================
 * MOTOR CONTROL
 * ========================================================================= */

// Motor Specs:
// Max Thrust (100%): 165g -> 1.62N
// Mid Thrust (50%):   63g -> 0.62N
//
// Linear model fails here (0.62 / 1.62 = 38%, but reality is 50%).
// We use a Power Law approximation: Cmd = (Force / Max)^0.714
#define MAX_THRUST_PER_MOTOR_N  1.62f 

float forceToVoltage(float forceNewtons) {
    if (forceNewtons <= 0.0f) return 0.0f;
    
    // Normalized Force (0.0 - 1.0 relative to max capability)
    float f_norm = forceNewtons / MAX_THRUST_PER_MOTOR_N;
    
    // Apply Power Law Curve to match 50% throttle point
    // Exponent 0.714 derived from 63g @ 0.5 cmd
    float cmd = powf(f_norm, 0.714f);
    
    if (cmd > 1.0f) cmd = 1.0f; 
    else if (cmd < 0.0f) cmd = 0.0f;
    
    return cmd;
}

// NEW: Map normalized command (0.0-1.0) to hardware duty cycle
float map_motor_signal(float cmd, int motor_idx) {
    if (cmd <= 0.0f) return 0.0f; // OFF
    
    // Scale: Output = Min + Cmd * (Max - Min)
    // This ensures 0.01 cmd -> Just barely spinning
    //              1.00 cmd -> Full power
    float min = MOTOR_START_DUTY[motor_idx];
    float max = MOTOR_MAX_DUTY;
    
    float mapped = min + cmd * (max - min);
    return mapped;
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

static float desHeight = 0.100f; // TARGET HEIGHT (Static so it remembers value)
static float last_height = 0.0f;
static uint32_t mission_timer_ms = 0;

void control_step(float dt) {

    // 1. SENSOR READ & PREDICTION
    read_imu_burst(dt);       // Updates Accel/Gyro + Predicts Z
    //printf("Finish read imu %d\n", clint_get_time(CLINT));
    //update_tof_fusion(dt);    // Corrects Z with ToF
    //printf("Finish ToF fusion %d\n", clint_get_time(CLINT));

    const float fc = 20.0f;  // Hz, start 10–30
    float alpha = (2.0f * 3.14159f * fc * dt) / (1.0f + 2.0f * 3.14159f * fc * dt);

    if (!lpf_inited) {
        ax_f = state.accelX;
        ay_f = state.accelY;
        az_f = state.accelZ;
        lpf_inited = 1;
    } else {
        ax_f += alpha * (state.accelX - ax_f);
        ay_f += alpha * (state.accelY - ay_f);
        az_f += alpha * (state.accelZ - az_f);
    }
    //printf("Accelerations LPF (m/s^2): X: %.3f Y: %.3f Z: %.3f\n", ax_f, ay_f, az_f);

    madgwick_update_imu(&filter, state.gyroX, state.gyroY, state.gyroZ,
                            ax_f, ay_f, az_f, dt);

    // 2. MADGWICK UPDATE
    //madgwick_update_imu(&filter, state.gyroX, state.gyroY, state.gyroZ, 
    //                        state.accelX, state.accelY, state.accelZ, dt);

    float q0=filter.q0, q1=filter.q1, q2=filter.q2, q3=filter.q3;
    float gx = 2.0f*(q1*q3 - q0*q2);
    float gy = 2.0f*(q0*q1 + q2*q3);
    float gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    //printf("g_body: %.2f %.2f %.2f\n", gx, gy, gz);

    // Convert Quaternion to Euler (Radians)
    float sinr_cosp = 2.0f * (filter.q0 * filter.q1 + filter.q2 * filter.q3);
    float cosr_cosp = 1.0f - 2.0f * (filter.q1 * filter.q1 + filter.q2 * filter.q2);
    state.estRoll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (filter.q0 * filter.q2 - filter.q3 * filter.q1);
    if (fabsf(sinp) >= 1.0f) state.estPitch = copysignf(M_PI / 2.0f, sinp);
    else state.estPitch = asinf(sinp);

    float siny_cosp = 2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2);
    float cosy_cosp = 1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3);
    state.estYaw = atan2f(siny_cosp, cosy_cosp);
    //printf("Finish Madgwick Step %d\n", clint_get_time(CLINT));

    /*printf("q:%.3f %.3f %.3f %.3f | g:%.2f %.2f %.2f | RPY:%.1f %.1f %.1f\n",
       filter.q0, filter.q1, filter.q2, filter.q3,
       gx, gy, gz,
       state.estRoll*57.2958f, state.estPitch*57.2958f, state.estYaw*57.2958f);*/

    // After madgwick_update() and after filter.q* are updated
    q0 = filter.q0, q1 = filter.q1, q2 = filter.q2, q3 = filter.q3;

    // Rotation matrix body->earth (R)
    // Earth-frame z component of a_body:
    //float aZ_e = 2.0f*(q1*q3 - q0*q2)*state.accelX
    //        + 2.0f*(q0*q1 + q2*q3)*state.accelY
    //        + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*state.accelZ;

    // subtract gravity to get "specific force" -> net vertical acceleration (m/s^2)
    //state.accelZ_earth = aZ_e - gravity;


    // 3. SAFETY CUTOFF (Crash Detection)
    //if (state.accelZ < -60.0f || state.accelZ > 60.0f) { 
    //    printf("Error: Excessive Vertical Accel! Az: %.2f\n", state.accelZ);
    //    error_flag = 1;
    //    return; 
    //}

    // --- SAFETY CUTOFFS ---
    // If tilt > 60 degrees (approx 1.0 rad), Kill motors.
    if (fabs(state.estRoll) > 1.0f || fabs(state.estPitch) > 1.0f) {
        printf("Error: Excessive Tilt! Roll: %.2f Pitch: %.2f\n", state.estRoll, state.estPitch); 
        error_flag = 1; 
        return;
    }

    /*if (state.estHeight < -0.5f || state.estHeight > 0.5f) {
        printf("Error: Unreasonable Height! H: %.2f\n", state.estHeight); 
        error_flag = 1; 
        return;
    }*/

    // 4. FLIGHT PLAN STATE MACHINE
    mission_timer_ms += (uint32_t)(dt * 1000);
    //printf("Mission Timer: %d ms\n", mission_timer_ms);
    if (mission_timer_ms < 10000) init_flag = 1;
    if (mission_timer_ms > 10000) descend_flag = 1;
    if (mission_timer_ms > 20000) {
        done_flag = 1;
        set_motors(0, 0, 0, 0);
        return;
    }

    // 5. UPDATE TARGET HEIGHT
    if (init_flag && !descend_flag) {
        // Hover Phase: Smoothly approach 0.75m if not there
        // (Optional: You can just leave it at 0.75 static)
        //desHeight = 0.10f; 
    }
    else if (descend_flag) {
        // Descent Phase: Decrease target by 0.2m per second
        //desHeight -= 0.02f * dt; 
        //if (desHeight < 0.05f) desHeight = 0.05f; // Floor at 5cm
        //if (desHeight < 0.01f) {
        //    done_flag = 1; // If we go below 1cm, consider mission complete
        //    set_motors(0, 0, 0, 0);
        //    return;
        //}
    }

    //float desAcc1 = -(1.0f / timeConst_horizVel) * state.estVel_1;
    //float desAcc2 = -(1.0f / timeConst_horizVel) * state.estVel_2;
    //float desRoll = -desAcc2 / gravity;
    //float desPitch = desAcc1 / gravity;
    //float desYaw = state.estYaw;

    // Determine "Airborne" state for Integral Logic
    // If we are above 5cm, or if motors are commanding > 20% thrust
    //int is_airborne = (state.estHeight > 0.02f);
    int is_airborne = 1;

    // 6. VERTICAL CONTROL (PID on Height + Vel)
    // Note: We use our CLEAN filtered velocity here!
    //printf("Start VERT PID Step %d\n", clint_get_time(CLINT));
    //const float desAcc3 = -2.0f * dampingRatio_height * natFreq_height * state.estVel_3 
    //                      - natFreq_height * natFreq_height * (state.estHeight - desHeight);
    
    //float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(state.estRoll) * cosf(state.estPitch));
    
    //if (desNormalizedAcceleration < 0) desNormalizedAcceleration = 0;
    //if (desNormalizedAcceleration > 2.0f * gravity) desNormalizedAcceleration = 2.0f * gravity;

    // 7. ATTITUDE CONTROL
    // Simple P-Controllers for Angle -> Rate -> Torque
    //printf("Start ANGLE PID Step %d\n", clint_get_time(CLINT));
    float desRoll = 0; // Level
    float desPitch = 0; // Level
    float desYaw = 0; // Lock Yaw (Simple)

    float rollRate_tgt = kp_angle * (desRoll - state.estRoll);
    float pitchRate_tgt = kp_angle * (desPitch - state.estPitch);
    //printf("pitch tgt: %.3f, roll tgt: %.3f\n", pitchRate_tgt, rollRate_tgt);
    
    // Shortest path yaw
    float yaw_err_ang = normalize_angle(desYaw - state.estYaw);
    float yawRate_tgt = kp_angle * yaw_err_ang;
    //printf("yaw tgt: %.3f\n", yawRate_tgt);

    // --- 3. RATE CONTROL (Inner Loop - PID) ---
    //printf("Start PID Inner Loop Step %d\n", clint_get_time(CLINT));
    /*printf("Errors Inner PID: Roll: %.3f, Pitch: %.3f, Yaw: %.3f\n", 
        rollRate_tgt - state.gyroX, 
        pitchRate_tgt - state.gyroY, 
        yawRate_tgt - state.gyroZ);*/
    float roll_torque  = pid_update(&pid_roll,  rollRate_tgt - state.gyroX, dt, kp_roll, ki_roll, kd_roll, is_airborne);
    float pitch_torque = pid_update(&pid_pitch, pitchRate_tgt - state.gyroY, dt, kp_pitch, ki_pitch, kd_pitch, is_airborne);
    float yaw_torque   = pid_update(&pid_yaw,   yawRate_tgt - state.gyroZ,  dt, kp_yaw, ki_yaw, kd_yaw, is_airborne);
    //printf("Finish PID Inner Loop Step %d\n", clint_get_time(CLINT));
    //printf("Torques: Pitch: %.4f\n", pitch_torque);

    // 8. MIXING
    //float u[4] = {desNormalizedAcceleration * mass, roll_torque, pitch_torque, yaw_torque};
    float u[4] = {gravity * mass, roll_torque, pitch_torque, yaw_torque};

    // ===== MIXER TEST MODE =====
    float T = mass * gravity;   // total thrust (N), not important for the pattern
    float tau = 0.01f;          // small torque (N*m) start tiny

    // Choose ONE test at a time:
    // Roll+
    //float u[4] = {T, +tau, 0.0f, 0.0f};

    // Pitch+
    //float u[4] = {T, 0.0f, +tau, 0.0f};

    // Yaw+
    //float u[4] = {T, 0.0f, 0.0f, +tau};


    float ctrl[4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ctrl[i] += M[i][j] * u[j];
        }
    }
    //printf("Control Outputs (N or N*m): M1: %.3f, M2: %.3f, M3: %.3f, M4: %.3f\n", ctrl[0], ctrl[1], ctrl[2], ctrl[3]);

    //printf("Finish Mixing Step %d\n", clint_get_time(CLINT));

    // Calculate Desired Throttle (0.0 to 1.0)
    float raw_cmds[4];
    raw_cmds[0] = forceToVoltage(ctrl[0]);
    raw_cmds[1] = forceToVoltage(ctrl[1]);
    raw_cmds[2] = forceToVoltage(ctrl[2]);
    raw_cmds[3] = forceToVoltage(ctrl[3]);

    //printf("Raw Motor Commands (0.0-1.0): M1: %.3f, M2: %.3f, M3: %.3f, M4: %.3f\n", 
    //    raw_cmds[0], raw_cmds[1], raw_cmds[2], raw_cmds[3]);

    // 9. OUTPUT
    if (init_flag && !done_flag && !error_flag) {
        // Enforce Min Throttle (Idle Spin) to prevent motor stall
        for(int i=0; i<4; i++) {
            if (raw_cmds[i] < 0.01f) raw_cmds[i] = 0.01f;
            if (raw_cmds[i] > 1.0f) raw_cmds[i] = 1.0f;
        }
        // APPLY HARDWARE MAPPING (The Fix)
        // This converts 0.05 command -> 0.58 Duty Cycle (Start Spin)
        motor_cmds[0] = map_motor_signal(raw_cmds[0], 0);
        motor_cmds[1] = map_motor_signal(raw_cmds[1], 1);
        motor_cmds[2] = map_motor_signal(raw_cmds[2], 2);
        motor_cmds[3] = map_motor_signal(raw_cmds[3], 3);

        set_motors(motor_cmds[0], motor_cmds[1], motor_cmds[2], motor_cmds[3]);
        //set_motors(0, 0, 0, 0);
        //printf("Finish Motor Set %d\n", clint_get_time(CLINT));
    } else {
        set_motors(0, 0, 0, 0);
    }
}

/* =========================================================================
 * INIT & MAIN
 * ========================================================================= */

void app_init() {
    GPIO_InitType gpio_init_config;
    gpio_init_config.mode = GPIO_MODE_OUTPUT;
    gpio_init_config.pull = GPIO_PULL_NONE;
    gpio_init_config.drive_strength = GPIO_DS_STRONG;
    gpio_init(GPIOC, &gpio_init_config, GPIO_PIN_0);
    gpio_init(GPIOC, &gpio_init_config, GPIO_PIN_1);

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

    pwm_enable(PWM0_BASE);
    pwm_set_frequency(PWM0_BASE, 0, MOT_FREQ_HZ);

    UART_InitType UART0_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_2};
    uart_init(UART0, &UART0_init_config);

    UART_InitType UART1_init_config = {9600, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART1, &UART1_init_config);

    I2C_InitType i2c_fmplus_conf;
    I2C_InitType i2c_fm_conf;
    i2c_fm_conf.clock = 400000;
    i2c_fmplus_conf.clock = 1000000;
    i2c_init(I2C0, &i2c_fmplus_conf);
    i2c_init(I2C1, &i2c_fm_conf);
    
    sensor_init_all();
    madgwick_init(&filter);

    //set_motors(0.1f, 0, 0, 0); 
    //msleep(250);
    //set_motors(0, 0.1f, 0, 0); 
    //msleep(250);
    //set_motors(0, 0, 0.1f, 0); 
    //msleep(250);
    //set_motors(0, 0, 0, 0.1f); 
    //msleep(250);
    //set_motors(0, 0, 0, 0);
    
    printf("Quad init done (Ardupilot Layout).\n");
}

void app_main() {
    printf("\n=== FLIGHT READY ===\n");
    printf("1. Place drone on level ground.\n");
    printf("2. Stand back.\n");
    printf(">> Press 'c' to ARM and START FLIGHT SEQUENCE <<\n");
    set_motors(0.38f, 0.38f, 0.38f, 0.38f);
    
    char c[1];
    uart_receive(UART1, c, 1, 10000000); // Wait forever (essentially)
    if (c[0] != 'c') return;

    init_flag = 0; descend_flag = 0; done_flag = 0;
    //desHeight = 0.10f;

    set_motors(0.38f, 0.38f, 0.38f, 0.38f); // Spin motors at idle to indicate ARMED
    msleep(5000);
    set_motors(0.55f, 0.55f, 0.55f, 0.55f);

    uint64_t last_control_us = get_time_us();
    uint64_t last_print_us = get_time_us();
    uint64_t now;

    // also reset PID memories to avoid weird first-step transients
    pid_roll.integral_err = pid_roll.last_err = 0.0f;
    pid_pitch.integral_err = pid_pitch.last_err = 0.0f;
    pid_yaw.integral_err = pid_yaw.last_err = 0.0f;

    while(1) {
        now = get_time_us();
        if ((now - last_control_us) >= 1000) {
            float dt = (float)(now - last_control_us) / 1000000.0f;
            last_control_us = now;
            //printf("Enter Control Step %d\n", clint_get_time(CLINT));
            control_step(dt);
            //printf("Exit Control Step %d\n", clint_get_time(CLINT));
            
             if (error_flag){
                gpio_write_pin(GPIOC, GPIO_PIN_0, 1);
                set_motors(0.0f, 0.0f, 0.0f, 0.0f);
                printf("ERROR DETECTED! Motors off...\n");
                break; 
            }
            if (done_flag) {
                set_motors(0.0f, 0.0f, 0.0f, 0.0f);
                printf("Flight Complete. Motors off...\n");
                break;
            } 
            if (init_flag) { gpio_write_pin(GPIOC, GPIO_PIN_1, 1); }
        }

        if ((now - last_print_us) >= 1000) {
            last_print_us = now;

            // SAFE PRINTING: Cast to Int to avoid Balloc crash
            int r_i = (int)(state.estRoll * 57.29f);
            int p_i = (int)(state.estPitch * 57.29f);
            int y_i = (int)(state.estYaw * 57.29f);
            //int h_cm = (int)(state.estHeight * 100.0f);
            //int vz_cm = (int)(state.estVel_3 * 100.0f);
            int m1 = (int)(motor_cmds[0] * 100);
            int m2 = (int)(motor_cmds[1] * 100);
            int m3 = (int)(motor_cmds[2] * 100);
            int m4 = (int)(motor_cmds[3] * 100);

            /*printf("P:%d R:%d Y:%d H:%dcm V:%dcm/s | M:%d %d %d %d\n", 
                   p_i, r_i, y_i, h_cm, vz_cm, m1, m2, m3, m4);*/
            printf("P:%d R:%d Y:%d | M:%d %d %d %d\n", 
                   p_i, r_i, y_i, m1, m2, m3, m4);
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