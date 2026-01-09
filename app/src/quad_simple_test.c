/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : quad_simple_test.c
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
#include <unistd.h> 
#include "main.h"
#include "chip_config.h"
#include "icm42688.h" 
#include "uart.h"

/* =========================================================================
 * SYSTEM CONFIGURATION
 * ========================================================================= */

/* =========================================================================
 * USER CONFIGURATION
 * ========================================================================= */

#define TEST_MOTOR_DUTY      0.15f 
#define TEST_DURATION_MS     10000
// PWM Frequency: 50Hz is the safest baseline for standard ESCs.
// Period = 20ms.
#define MOT_FREQ_HZ         381
#define PWM_PERIOD_US       2624
#define PWM_PERIOD_MS       2.624f

// Duty Cycle Calculation for 50Hz:
// 1ms pulse (0% Throttle)  = 1ms / 20ms = 5% Duty Cycle
// 2ms pulse (100% Throttle) = 2ms / 20ms = 10% Duty Cycle
#define PWM_DUTY_MIN_THROTTLE 38  // 1ms pulse
#define PWM_DUTY_MAX_THROTTLE 76 // 2ms pulse

#define PULSE_MIN     1000 // Arming / Zero
#define PULSE_SPIN    1250 // Idle Spin (~51% duty)
#define PULSE_MAX     2000 // Full Throttle
#define TIME_CALIB_FACTOR 1.47f

// Test Power: 0.15 = 15% Throttle (Should be enough to spin up visibly)
// CAUTION: Ensure props are OFF for first test!
#define TEST_THROTTLE_PERCENT 0.25f 
#define TEST_DURATION_MS      1000

#define MOTOR1_PWM_CH 0
#define MOTOR2_PWM_CH 1
#define MOTOR3_PWM_CH 2
#define MOTOR4_PWM_CH 3

/**
 * @brief Sets motor throttle considering ESC calibration range.
 * @param channel PWM Channel
 * @param throttle 0.0 (Stop) to 1.0 (Full Power)
 */

void stop_all_motors() {
    printf("Stopping all motors...\n");
    set_all_motors_raw_duty(0);
}

// Helper to update all motors with raw Integer duty (0-100)
// This bypasses the float math errors in your original 'set_motor_throttle'
void set_all_motors_raw_duty(uint32_t duty) {
    pwm_set_duty_cycle(PWM0_BASE, MOTOR1_PWM_CH, duty, 0); //MOT 3
    pwm_set_duty_cycle(PWM0_BASE, MOTOR2_PWM_CH, duty, 0); //MOT 1
    pwm_set_duty_cycle(PWM0_BASE, MOTOR3_PWM_CH, duty, 0); //MOT 2
    pwm_set_duty_cycle(PWM0_BASE, MOTOR4_PWM_CH, duty, 0); //MOT 4
}

void handle_sigint(int sig) { 
    stop_all_motors(); 
}

static ICM42688_Data imu_data;

void app_init() {

    GPIO_InitType gpio_init_config;
    gpio_init_config.mode = GPIO_MODE_OUTPUT;
    gpio_init_config.pull = GPIO_PULL_NONE;
    gpio_init_config.drive_strength = GPIO_DS_STRONG;
    gpio_init(GPIOC, &gpio_init_config, GPIO_PIN_7);
    gpio_init(GPIOC, &gpio_init_config, GPIO_PIN_9);
    gpio_write_pin(GPIOC, GPIO_PIN_7, 0);
    gpio_write_pin(GPIOC, GPIO_PIN_9, 0);
    

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
    printf("Frequency %d\n", pwm_get_frequency(PWM0_BASE, 0));

    UART_InitType UART0_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_2};
    uart_init(UART0, &UART0_init_config);

    UART_InitType UART1_init_config = {115200, UART_MODE_TX_RX, UART_STOPBITS_1};
    uart_init(UART1, &UART1_init_config);

    I2C_InitType i2c_conf;
    i2c_conf.clock = 400000;
    i2c_init(I2C0, &i2c_conf);

    //QSPI0->SCKDIV = SYS_CLK_FREQ / (2 * 1000000) - 1;

    //set_all_clocks(CLOCK_SELECTOR_BASE, 0);
    //configure_pll(PLL, 10, 0);
    //set_all_clocks(CLOCK_SELECTOR_BASE, 1);
    
    printf("\n\n=== Quadcopter Hardware Test (Ardupilot Layout) ===\n");
}

void app_main() {
    printf("[1/2] Checking IMU (ICM-42688)...\n");
    
    if (icm42688_init(I2C0, CLINT) != 0) {
        printf("ERROR: IMU Init Failed.\n");
        while(1); 
    }

    if (icm42688_read_all(I2C0, &imu_data, CLINT) != 0) {
        printf("ERROR: IMU Read Failed.\n");
        while(1);
    }

    if (imu_data.accel_z == 0 && imu_data.accel_x == 0) {
        printf("WARNING: IMU data is all zeros.\n");
    } else {
        printf("SUCCESS: IMU Valid. Accel X: %d\n, Accel Z: %d\n", imu_data.accel_x, imu_data.accel_z);
    }
    
    msleep(1000);

    printf("\n=== SAFETY MOTOR TEST START ===\n");

    // Wait for user interaction to confirm they hear the beeping
    printf(">> Press 'c' to drop throttle and ARM motors <<\n");
    char c[1];
    uart_receive(UART0, c, 1, 100000); // Blocking wait
    if (c[0] != 'c') return;

    // ---------------------------------------------------------
    // STEP 2: ARMING (Drop to Min Throttle)
    // ---------------------------------------------------------
    // 1000us at 381Hz = ~38% Duty Cycle.
    // Effect: Beeping stops. ESCs play "Low-High" tones.
    //printf("2. Dropping to MIN THROTTLE (1000us / 38%%)...\n");
    //printf("   Expected: 'Musical Tone' (Arming Sequence).\n");
    current_pulse_us = PULSE_MIN;
    set_all_motors_raw_duty(38);
    //soft_pwm_loop(5000);
    
    // Give the ESCs 2 seconds to initialize the zero-point
    //printf("   Arming... (Waiting 7s)\n");
    msleep(7000);

    // ---------------------------------------------------------
    // STEP 3: IDLE SPIN (The 'AirMode' Bump)
    // ---------------------------------------------------------
    // We must jump UP to ~1100us to overcome friction and deadband.
    // 1100us at 381Hz = ~42% Duty Cycle.
    printf("3. Spinning at IDLE (1100us / 42%%)...\n");
    //printf("   MOTORS SHOULD SPIN NOW.\n");
    current_pulse_us = PULSE_SPIN;
    set_all_motors_raw_duty(58);
    //soft_pwm_loop(5000);
    
    // Spin for 3 seconds
    msleep(3000);

    // ---------------------------------------------------------
    // STEP 4: SHUTDOWN
    // ---------------------------------------------------------
    printf("4. Test Complete. Disarming.\n");
    current_pulse_us = 1050;
    pwm_set_duty_cycle(PWM0_BASE, MOTOR3_PWM_CH, 58, 0);
    pwm_set_duty_cycle(PWM0_BASE, MOTOR1_PWM_CH, 55, 0);
    pwm_set_duty_cycle(PWM0_BASE, MOTOR2_PWM_CH, 70, 0);
    pwm_set_duty_cycle(PWM0_BASE, MOTOR4_PWM_CH, 64, 0);

    printf(">> Press 's' to drop throttle and DISARM motors <<\n");
    while(1) {
        char s[1];
        uart_receive(UART0, c, 1, 100000); // Blocking wait
        if (s[0] != 's')
        {
            set_all_motors_raw_duty(38);
            return;
        }
    }
}

int main(int argc, char **argv) {
    app_init();
    signal(SIGINT, handle_sigint);
    app_main();
    return 0;
}

void __attribute__((weak, noreturn)) __main(void) {
  uint64_t mhartid = READ_CSR("mhartid");
  while (1) {
    asm volatile("wfi");
  }
}