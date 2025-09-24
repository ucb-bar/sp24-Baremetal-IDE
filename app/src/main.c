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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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
  GPIO_InitType gpio_init_config;
  gpio_init_config.mode = GPIO_MODE_INPUT;
  gpio_init_config.pull = GPIO_PULL_NONE;
  gpio_init_config.drive_strength = GPIO_DS_STRONG;

  GPIO_InitType gpio2_init_config;
  gpio2_init_config.mode = GPIO_MODE_OUTPUT;
  gpio_init_config.pull = GPIO_PULL_NONE;
  gpio_init_config.drive_strength = GPIO_DS_STRONG;

  gpio_init(GPIOC, &gpio2_init_config, GPIO_PIN_0);
  gpio_init(GPIOC, &gpio2_init_config, GPIO_PIN_1);
  gpio_init(GPIOC, &gpio2_init_config, GPIO_PIN_2);
  gpio_init(GPIOC, &gpio2_init_config, GPIO_PIN_3);

  pwm_enable(PWM0_BASE);
  pwm_set_frequency(PWM0_BASE, 0, 3000);
  pwm_get_frequency(PWM0_BASE, 0);
  pwm_set_duty_cycle(PWM0_BASE, 0, 10, 0);
  pwm_get_duty_cycle(PWM0_BASE, 0);
  pwm_set_duty_cycle(PWM0_BASE, 1, 50, 0);
  pwm_get_duty_cycle(PWM0_BASE, 1);
  pwm_set_duty_cycle(PWM0_BASE, 2, 70, 0);
  pwm_get_duty_cycle(PWM0_BASE, 2);
  pwm_set_duty_cycle(PWM0_BASE, 3, 30, 0);
  pwm_get_duty_cycle(PWM0_BASE, 3);

  
  QSPI0->SCKDIV = SYS_CLK_FREQ / (2 * 1000000) - 1;

  set_all_clocks(CLOCK_SELECTOR_BASE, 0);
  configure_pll(PLL, 9, 0);
  set_all_clocks(CLOCK_SELECTOR_BASE, 1);
}

void handle_sigint(int sig) {
  printf("\nCaught signal %d, exiting...\n", sig);
  //exit(0);
}

void app_main() {
  while (1) {
    gpio_write_pin(GPIOC, GPIO_PIN_0, 0);
    gpio_write_pin(GPIOC, GPIO_PIN_1, 1);
    gpio_write_pin(GPIOC, GPIO_PIN_2, 1);
    gpio_write_pin(GPIOC, GPIO_PIN_3, 0);
    printf("First LEDs\n"); 
    /*Always need a new line after print statement 
    to ensure it doesn't try to fill buffer and 
    prints every new line*/
    msleep(1000);
    gpio_write_pin(GPIOC, GPIO_PIN_0, 1);
    gpio_write_pin(GPIOC, GPIO_PIN_1, 0);
    gpio_write_pin(GPIOC, GPIO_PIN_2, 0);
    gpio_write_pin(GPIOC, GPIO_PIN_3, 1);
    printf("Second LEDs\n");
    msleep(1000);
  }
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
  /* USER CODE BEGIN SysInit */
  // Initialize UART0 for Serial Monitor
  UART_InitType UART0_init_config;
  UART0_init_config.baudrate = 115200;
  UART0_init_config.mode = UART_MODE_TX_RX;
  UART0_init_config.stopbits = UART_STOPBITS_2;
  uart_init(UART0, &UART0_init_config);

  int uart_divisor = (SYS_CLK_FREQ / 115200) - 1;

  UART_InitType UART1_init_config;
  UART1_init_config.baudrate = 115200;
  UART1_init_config.mode = UART_MODE_TX_RX;
  UART1_init_config.stopbits = UART_STOPBITS_1;
  uart_init(UART1, &UART1_init_config);

  // Initialize PWM0 for Motor Control
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
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN Init */
  app_init();
  signal(SIGINT, handle_sigint);
  /* USER CODE END Init */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    app_main();
  }
  return 0;
  /* USER CODE END WHILE */
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