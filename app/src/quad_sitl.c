/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : quad_sitl.c
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
#include "main.h"
#include "chip_config.h"

/* =========================================================================
 * MOCK CONFIG & CONSTANTS (Copied from quad.c)
 * ========================================================================= */

const float mass = 260e-3f;
const float gravity = 9.81f;
const float l = 88.9e-3f; // Arm length
const float k = 0.01f;  // Torque constant ratio

// PID Constants
const float tau_roll = 0.10f;
const float tau_pitch = 0.10f;
const float tau_yaw = 0.25f;
const float tau_rollRate = 0.025f;
const float tau_pitchRate = 0.025f;
const float tau_yawRate = 0.05f;
const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

// MIXING MATRIX (Corrected for M1=FR, M2=BL, M3=FL, M4=RR)
// Rows: 0=M1, 1=M2, 2=M3, 3=M4
// Cols: Thrust, Roll, Pitch, Yaw
// Signs:
// Roll:  Right(-), Left(+)
// Pitch: Front(+), Back(-)
// Yaw:   CCW(+),   CW(-)  <-- Assuming M1/M2 are CCW, M3/M4 are CW
const float M[4][4] = {
    // Thrust, Roll,      Pitch,     Yaw
    {0.25f,   -0.25f/l,   0.25f/l,   0.25f/k}, // M1: Front Right (R-, P+, Y+)
    {0.25f,    0.25f/l,  -0.25f/l,   0.25f/k}, // M2: Back Left   (R+, P-, Y+)
    {0.25f,    0.25f/l,   0.25f/l,  -0.25f/k}, // M3: Front Left  (R+, P+, Y-)
    {0.25f,   -0.25f/l,  -0.25f/l,  -0.25f/k}  // M4: Back Right  (R-, P-, Y-)
};

// MOMENT OF INERTIA (J)
// Calculated based on 260g total mass, 7.15g motors, and measured hub dimensions.
const float J[3][3] = {
    {0.000370f, 0, 0}, // Ixx (Roll Inertia)
    {0, 0.000239f, 0}, // Iyy (Pitch Inertia - Lower because body is narrower in Width)
    {0, 0, 0.000497f}  // Izz (Yaw Inertia)
};

/* =========================================================================
 * UTILS
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

/* =========================================================================
 * TEST LOGIC
 * ========================================================================= */

// This function runs the exact control logic from quad.c but with INJECTED state
void run_simulation_step(const char* test_name, float roll, float pitch, float yaw) {
    
    printf("\n--- TEST: %s ---\n", test_name);
    printf("Input State -> Roll: %.2f, Pitch: %.2f\n", roll, pitch);

    // 1. MOCK STATE
    float estRoll = roll;
    float estPitch = pitch;
    float estYaw = yaw;
    float estHeight = 0.75f; // Already at target height
    float estVel_3 = 0.0f;
    float gyroX = 0; float gyroY = 0; float gyroZ = 0;

    // 2. CONTROL LOGIC (Copied from quad.c)
    float desHeight = 0.75f; 
    float desAcc1 = 0; // Assume stationary
    float desAcc2 = 0;
    
    float desRoll = -desAcc2 / gravity;
    float desPitch = desAcc1 / gravity;
    float desYaw = 0; 

    const float desAcc3 = -2.0f * dampingRatio_height * natFreq_height * estVel_3 
                          - natFreq_height * natFreq_height * (estHeight - desHeight);
    
    float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

    float rollRate_tgt = (-1.0f / tau_roll) * (estRoll - desRoll);
    float pitchRate_tgt = (-1.0f / tau_pitch) * (estPitch - desPitch);
    float yawRate_tgt = (-1.0f / tau_yaw) * (estYaw - desYaw);

    float rollRate_cmd = (-1.0f / tau_rollRate) * (gyroX - rollRate_tgt);
    float pitchRate_cmd = (-1.0f / tau_pitchRate) * (gyroY - pitchRate_tgt);
    float yawRate_cmd = (-1.0f / tau_yawRate) * (gyroZ - yawRate_tgt);

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

    // 3. PRINT OUTPUTS
    float m1 = forceToVoltage(0.9f * ctrl[0]);
    float m2 = forceToVoltage(0.9f * ctrl[1]);
    float m3 = forceToVoltage(0.9f * ctrl[2]);
    float m4 = forceToVoltage(0.9f * ctrl[3]);

    printf("Motors (0.0-1.0): M1:%f  M2:%f  M3:%f  M4:%f\n", m1, m2, m3, m4);
    
    // 4. ANALYSIS
    // Assuming Standard Quad X config:
    // M1: Front Right, M2: Rear Left, M3: Rear Right, M4: Front Left (Check your specific frame!)
    // IF TILTED RIGHT (Roll > 0): Right motors (M1, M3) should be LOWER than Left motors (M2, M4).
}

void app_init() {
    // Setup UART0 (Standard Output / printf)
    UART_InitType UART0_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_2};
    uart_init(UART0, &UART0_init_config);

    // Setup UART1 (Telemetry / Aux)
    UART_InitType UART1_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART1, &UART1_init_config);
    
    printf("\n=== Mixer Logic Validation ===\n");
}

void app_main() {
    // Case 1: Level Hover
    // All motors should be roughly equal and providing lift.
    run_simulation_step("HOVER (Level)", 0.0f, 0.0f, 0.0f);

    // Case 2: Tilted RIGHT (Positive Roll)
    // Controller wants to roll LEFT.
    // Right motors (M1, M4) must SPEED UP (Increase lift).
    // Left motors (M2, M3) must SLOW DOWN.
    run_simulation_step("TILTED RIGHT (+0.2 rad)", 0.2f, 0.0f, 0.0f);

    // Case 3: Tilted NOSE UP (Positive Pitch)
    // Controller wants to pitch DOWN.
    // Rear motors (M2, M4) must SPEED UP.
    // Front motors (M1, M3) must SLOW DOWN.
    run_simulation_step("NOSE UP (+0.2 rad)", 0.0f, 0.2f, 0.0f);

    printf("\nCheck these values against your frame layout before flying!\n");
    while(1) { asm volatile("wfi"); }
}

/* =========================================================================
 * SYSTEM ENTRIES
 * ========================================================================= */

int main(int argc, char **argv) {
    app_init();
    app_main();
    return 0;
}

void handle_sigint(int sig) {}

void __attribute__((weak, noreturn)) __main(void) {
  uint64_t mhartid = READ_CSR("mhartid");
  while (1) {
    asm volatile("wfi");
  }
}