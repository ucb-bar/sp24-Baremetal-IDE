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
#include "uart.h"

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

const float mass = 35e-3f;
const float gravity = 9.81f;
const float l = 33e-3f; 
const float k = 0.01f;

const float tau_roll = 0.10f;
const float tau_pitch = 0.10f;
const float tau_yaw = 0.25f;
const float tau_rollRate = 0.025f;
const float tau_pitchRate = 0.025f;
const float tau_yawRate = 0.05f;
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

// NEW MIXING MATRIX (Ardupilot Quad-X)
const float M[4][4] = {
    {0.25f, -0.25f/l, -0.25f/l,  0.25f/k}, // M1: FR (Right, Front, CCW)
    {0.25f,  0.25f/l,  0.25f/l,  0.25f/k}, // M2: RL (Left, Rear, CCW)
    {0.25f,  0.25f/l, -0.25f/l, -0.25f/k}, // M3: FL (Left, Front, CW)
    {0.25f, -0.25f/l,  0.25f/l, -0.25f/k}  // M4: RR (Right, Rear, CW)
};

const float J[3][3] = {
    {16e-6f, 0, 0}, {0, 16e-6f, 0}, {0, 0, 29e-6f}
};

/* =========================================================================
 * UTILS
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

/* =========================================================================
 * TEST LOGIC
 * ========================================================================= */

void run_simulation_step(const char* test_name, float roll, float pitch, float yaw) {
    printf("\n--- TEST: %s ---\n", test_name);
    printf("Input State -> Roll: %.2f, Pitch: %.2f\n", roll, pitch);

    float estRoll = roll; float estPitch = pitch; float estYaw = yaw;
    float estHeight = 0.75f; float estVel_3 = 0.0f;
    float gyroX = 0; float gyroY = 0; float gyroZ = 0;

    float desHeight = 0.75f; float desAcc1 = 0; float desAcc2 = 0;
    float desRoll = -desAcc2 / gravity; float desPitch = desAcc1 / gravity; float desYaw = 0; 

    const float desAcc3 = -2.0f * dampingRatio_height * natFreq_height * estVel_3 - natFreq_height * natFreq_height * (estHeight - desHeight);
    float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

    float rollRate_tgt = (-1.0f / tau_roll) * (estRoll - desRoll);
    float pitchRate_tgt = (-1.0f / tau_pitch) * (estPitch - desPitch);
    float yawRate_tgt = (-1.0f / tau_yaw) * (estYaw - desYaw);

    float rollRate_cmd = (-1.0f / tau_rollRate) * (gyroX - rollRate_tgt);
    float pitchRate_cmd = (-1.0f / tau_pitchRate) * (gyroY - pitchRate_tgt);
    float yawRate_cmd = (-1.0f / tau_yawRate) * (gyroZ - yawRate_tgt);

    float u[4] = {desNormalizedAcceleration * mass, 0, 0, 0};
    u[1] = rollRate_cmd * J[0][0]; u[2] = pitchRate_cmd * J[1][1]; u[3] = yawRate_cmd * J[2][2];

    float ctrl[4] = {0};
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) ctrl[i] += M[i][j] * u[j];

    float m1 = forceToVoltage(0.9f * ctrl[1]);
    float m2 = forceToVoltage(0.9f * ctrl[2]);
    float m3 = forceToVoltage(0.9f * ctrl[3]);
    float m4 = forceToVoltage(0.9f * ctrl[0]);

    printf("Motors: M1:%.2f M2:%.2f M3:%.2f M4:%.2f\n", m1, m2, m3, m4);
}

void app_init() {
    UART_InitType UART0_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_2};
    uart_init(UART0, &UART0_init_config);

    UART_InitType UART1_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART1, &UART1_init_config);
    
    printf("\n=== Mixer Logic Validation (Ardupilot Layout) ===\n");
}

void app_main() {
    run_simulation_step("HOVER", 0.0f, 0.0f, 0.0f);

    // Case 2: Tilted RIGHT (Positive Roll)
    // Drone detects Right Tilt. Must roll Left to correct.
    // Left Motors (M2, M3) should be HIGHER than Right Motors (M1, M4).
    run_simulation_step("TILTED RIGHT (+0.2 rad)", 0.2f, 0.0f, 0.0f);

    // Case 3: Tilted NOSE UP (Positive Pitch)
    // Drone detects Nose Up. Must pitch Down to correct.
    // Rear Motors (M2, M4) should be HIGHER than Front Motors (M1, M3).
    run_simulation_step("NOSE UP (+0.2 rad)", 0.0f, 0.2f, 0.0f);

    printf("\nCheck values match layout logic!\n");
    while(1) { asm volatile("wfi"); }
}

int main(int argc, char **argv) { app_init(); app_main(); return 0; }
void handle_sigint(int sig) {}
void __attribute__((weak, noreturn)) __main(void) { while (1) asm volatile("wfi"); }