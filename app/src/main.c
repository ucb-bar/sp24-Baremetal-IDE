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


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "chip_config.h"
#include "bmi088.h"
#include <math.h>
#include <time.h>
#include <controller.h>

int motor_speeds[8];
int motor_positions[8];

/* USER CODE BEGIN PV */
const unsigned char *ASCII_CRLF = (const unsigned char *) "\r\n";
const unsigned char *ASCII_BEL = (const unsigned char *) "\a";

void read_stdin(const char* guide, char* buffer, size_t bufsize) {
  // read a line from stdin, up to but not including \n
  // printf("%s", guide);
  // if (fgets(buffer, bufsize, stdin) != NULL) {
  //     size_t len = strlen(buffer);
  //     if (len > 0 && buffer[len - 1] == '\n') {
  //         buffer[len - 1] = '\0'; // strip newline
  //     }
  // }
  size_t char_offset = 0;
  size_t upper_bufsize_bound = bufsize - 1;
  printf("%s", guide);
  fflush(stdout);

  while(1) {
    unsigned char input_char = '\0';
    uart_receive(UART0, &input_char, 1, 100);

    if (input_char == '\b' && char_offset > 0) {
      // Backspace handling
      uart_transmit(UART0, &input_char, 1, 100);
      char_offset--;
      *(buffer + char_offset) = '\0';
    } else if (input_char == '\r') {
      // Newline (submit) handling
      uart_transmit(UART0, ASCII_CRLF, 2, 100);
      *(buffer + char_offset) = '\0';
      break;
    } else if (input_char > 31 && input_char < 127 && char_offset < upper_bufsize_bound) {
      // Any printable ascii character
      uart_transmit(UART0, &input_char, 1, 100);
      *(buffer + char_offset) = input_char;
      char_offset++;
    } else {
      // Send a bell to the terminal if some other condition.
      uart_transmit(UART0, ASCII_BEL, 1, 100);
    }
  }
}

#include "switch.h"
#include "joints.h"
#include "controller.h"

#define F_LOW 530
#define F_HIGH 1000

#define F_FRONT 150
#define F_BACK 800

#define B_FRONT 800
#define B_BACK 150

// Forward gait
int step_fwd[6][8] = {
    {F_LOW, F_FRONT, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
    {F_LOW, F_BACK, F_HIGH, B_FRONT, F_FRONT, F_HIGH, B_BACK, F_LOW},
    {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
    {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
    {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
    {F_LOW, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
};

// Backward gait (mirror of forward)
int step_back[6][8] = {
    {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
    {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
    {F_LOW, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
    {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
    {F_LOW, F_BACK, F_HIGH, B_FRONT, F_FRONT, F_HIGH, B_BACK, F_LOW},
    {F_LOW, F_FRONT, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
};

// Left turn gait (exaggerated left side backward, right side forward)
int turn_left_gait[6][8] = {
    {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
    {F_HIGH, F_BACK, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
    {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
    {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
    {F_HIGH, F_BACK, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
    {F_LOW, F_BACK, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
};

// Right turn gait (mirror of left)
int turn_right_gait[6][8] = {
    {F_LOW, F_FRONT, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
    {F_LOW, F_BACK, F_HIGH, B_FRONT, F_FRONT, F_HIGH, B_BACK, F_LOW},
    {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
    {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
    {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
    {F_LOW, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
};

// Perform a full gait step (6-frame motion)
void step_with_gait(int gait[6][8]) {
    for (int t = 0; t < 6; t++) {
        for (int i = 0; i < 8; i++) {
            set_motor_pos(i, gait[t][i]);
        }
        msleep(1000);
    }
}

// CLI command handler
void controller_cli(char *cli_user_prompt, int steps) {
    char user_prompt[16];
    printf("Hi, my name is George! What shall I do now? [front, left, right, back, dance, exit]\r\n");

    srand(time(NULL)); // Initialize RNG

    while (1) {
        // Read input from function argument (first time only)
        if (cli_user_prompt != NULL) {
            strncpy(user_prompt, cli_user_prompt, sizeof(user_prompt) - 1);
            user_prompt[sizeof(user_prompt) - 1] = '\0';
            cli_user_prompt = NULL;
        } else {
            read_stdin("User: ", user_prompt, sizeof(user_prompt));
        }

        if (strcmp(user_prompt, "front") == 0) {
            printf("Stepping forward!\r\n");
            for (int i = 0; i < steps; i++) step_with_gait(step_fwd);

        } else if (strcmp(user_prompt, "back") == 0) {
            printf("Stepping backward!\r\n");
            for (int i = 0; i < steps; i++) step_with_gait(step_back);

        } else if (strcmp(user_prompt, "left") == 0) {
            printf("Turning left!\r\n");
            for (int i = 0; i < steps; i++) step_with_gait(turn_left_gait);

        } else if (strcmp(user_prompt, "right") == 0) {
            printf("Turning right!\r\n");
            for (int i = 0; i < steps; i++) step_with_gait(turn_right_gait);

        } else if (strcmp(user_prompt, "dance") == 0) {
            printf("Dance party!\r\n");
            for (int i = 0; i < 5; ++i) {
                for (int m = 0; m < 8; ++m) {
                    int pos = (rand() % 2) ? F_HIGH : F_BACK;
                    set_motor_pos(m, pos);
                }
                msleep(300);
            }

        } else if (strcmp(user_prompt, "exit") == 0 || strcmp(user_prompt, "quit") == 0) {
            printf("Exiting...\r\n");
            break;

        } else {
            printf("Unknown command: '%s'\r\n", user_prompt);
        }
    }

    printf("Okay - bye bye!\r\n");
}

void app_init() {

  // +------------------------------------------------+
  // | Initialize the controller
  // +------------------------------------------------+
  printf("[START INIT]\r\n");
  switches_init();
  joints_init();

  // +------------------------------------------------+
  // | Start initial homing sequence
  // +------------------------------------------------+
  printf("[START HOME]\r\n");
  home_motors();    

  sleep(1);
  
}

void app_main() {
  uint64_t mhartid = READ_CSR("mhartid");

  // DEMO BASIC
  // Moves 
  // while (1) {
  //   set_motor_pos(7, 0);
  //   msleep(3000);
  //   set_motor_pos(7, 500);
  //   msleep(3000);
  // }

  // DEMO TWO
  // step_og();
  //controller_cli("front", 5);

  // DEMO THREE
  for (int t = 0; t < 3; t++) {
    int step_fwd_test[6][8] = {
        // {F_LOW, F_FRONT, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
        // {F_LOW, F_BACK, F_HIGH, B_FRONT, F_FRONT, F_HIGH, B_BACK, F_LOW},
        // {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
        // {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
        // {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
        // {F_LOW, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
        {0, 0, 0, 500, 0, 0, 0, 500},
        {0, 0, 0, 0, 0, 0, 0, 500},
        {0, 0, 0, 500, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 500},
        {0, 0, 0, 500, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 500},
    };
    step_with_gait(step_fwd_test);
  }
  for (int t = 0; t < 3; t++) {
    int step_fwd_test[6][8] = {
        // {F_LOW, F_FRONT, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
        // {F_LOW, F_BACK, F_HIGH, B_FRONT, F_FRONT, F_HIGH, B_BACK, F_LOW},
        // {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
        // {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
        // {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
        // {F_LOW, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        {-500, 0, 0, 500, 0, 0, 0, 0},
        {-500, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 500, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        {-500, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 500, 0, 0, 0, 0},
        {-500, 0, 0, 0, 0, 0, 0, 0},
    };
    step_with_gait(step_fwd_test);
  }
  for (int t = 0; t < 3; t++) {
    int step_fwd_test[6][8] = {
        // {F_LOW, F_FRONT, F_HIGH, B_BACK, F_BACK, F_HIGH, B_FRONT, F_LOW},
        // {F_LOW, F_BACK, F_HIGH, B_FRONT, F_FRONT, F_HIGH, B_BACK, F_LOW},
        // {F_LOW, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_LOW},
        // {F_HIGH, F_BACK, F_LOW, B_FRONT, F_FRONT, F_LOW, B_BACK, F_HIGH},
        // {F_HIGH, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_HIGH},
        // {F_LOW, F_FRONT, F_LOW, B_BACK, F_BACK, F_LOW, B_FRONT, F_LOW},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        {-500, 0, 0, 0, 0, 500, 0, 0},
        {-500, 0, 500, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 500, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        // {0, 0, 0, 0, 0, 0, 0, 0},
        {-500, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, -500, 0, 0, 500, 0, 0},
        {-500, 0, 0, 0, 0, 500, 0, 0},
    };
    step_with_gait(step_fwd_test);
  }

  // while (1) {
  //   // step();
  //   // print_home_buttons();
  //   // print_encoders();
  //   // printf("sadly unalive myself from hart : %d\r\n", mhartid);
  //   // msleep(100);

  //   // for(int i = 0; i < 8; i++) {
  //   //   set_motor_pos(i, 0);
  //   // }

  //   set_motor_pos(3, 0);
  //   // set_motor_pos(1, 0);
  //   // set_motor_pos(2, 0);
  //   // set_motor_pos(3, 0);
  //   // set_motor_pos(4, 0);
  //   // set_motor_pos(5, 0);
  //   // set_motor_pos(6, 0);
  //   // set_motor_pos(7, 0);

  //   msleep(3000);
  //   set_motor_pos(3, 500);
  //   msleep(1000);
  //   set_motor_pos(1, 500);
  //   // set_motor_pos(2, 500);
  //   msleep(1000);
  //   // set_motor_pos(3, 500);
  //   set_motor_pos(4, 500);
  //   msleep(1000);
  //   set_motor_pos(6, 500);

  //   // for(int i = 0; i < 8; i++) {
  //   //   set_motor_pos(i, 500);
  //   // }

  //   msleep(3000);
  // }
}


void print_encoders() {
  int enc[8];

  for (int i = 0; i < 8; i++) {
    enc[i] = get_encoder(i);
  }

  printf("[%d, %d, %d, %d, %d, %d, %d, %d]\n", enc[0], enc[1], enc[2], enc[3], enc[4], enc[5], enc[6], enc[7]);
}

void print_home_buttons() {
  printf("[%d, %d, %d, %d, %d, %d, %d, %d]\n", read_switch(0), read_switch(1), read_switch(2), read_switch(3), read_switch(4), read_switch(5), read_switch(6), read_switch(7));
}

void passthrough_speeds() {
  for (int i = 0; i < 8; i++) {
    set_motor_speed(i, motor_speeds[i]);
  }
}

void passthrough_positions() {
  for (int i = 0; i < 8; i++) {
    set_motor_pos(i, motor_positions[i]);
  }
}



void setup_pll() {
  printf("Finished setting up PLL\r\n");
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(int argc, char **argv) {
  /* MCU Configuration--------------------------------------------------------*/
  // Initialize UART0 for Serial Monitor
  UART_InitType UART0_init_config;
  UART0_init_config.baudrate = 115200;
  UART0_init_config.mode = UART_MODE_TX_RX;
  UART0_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART0_init_config);


  // /* Initialize the PLL so that we can run at 500MHz */
  // setup_pll();
  sleep(2);


  /* Initialize all configured peripherals */
  printf("-----Initialize App-----\r\n");
  app_init();

  /* Infinite loop */
  printf("-----Start Main-----\r\n");
  while (1) {
    app_main();
  }
  return 0;
}

/*
 * Main function for secondary harts
 *
 * Multi-threaded programs should provide their own implementation.
 */
void __attribute__((weak, noreturn)) __main(void) {
  uint64_t mhartid = READ_CSR("mhartid");
  while (1) {
    asm volatile("wfi");
  }
}