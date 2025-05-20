// https://github.com/nyameaama/BMI088-Driver-ESP-IDF/blob/main/bmi088.cpp
#include "bmi088.h"
#include <stdio.h>
#include <stdint.h>
#include "chip_config.h"
#include "main.h"


#define bmi088_ACCEL_ADDRESS                 0x18        /*!< Slave address of the bmi088_acceleromter sensor SD01 pull to GND */
#define bmi088_GYRO_ADDRESS                  0x69        /*!< Slave address of the bmi088 gyroscope sensor SD02 pull to GND*/
int16_t ret = 0;

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

static const char *TAG = "bmi088 Module";

/**
 * @brief Memory locations to store bmi088 accel and gyro Sensors data
 */
uint8_t accel_x[2] = {0};
uint8_t accel_y[2] = {0};
uint8_t accel_z[2] = {0};

uint8_t accel_full[6] = {0};
uint8_t gyro_full[6] = {0};

uint8_t gyro_x[2] = {0};
uint8_t gyro_y[2] = {0};
uint8_t gyro_z[2] = {0};

/**
 * @brief Convertion of bmi088 accel and gyro Sensors data into signed integer
 */
float accel_x_int16 = 0;
float accel_y_int16 = 0;
float accel_z_int16 = 0;


 float gyro_x_int16 = 0;
 float gyro_y_int16 = 0;
 float gyro_z_int16 = 0;


/**
 * @brief Convertion of bmi088 accel and gyro Sensors  signed integer into acceleration
 */
float accel_x_in_mg = 0;
float accel_y_in_mg = 0;
float accel_z_in_mg = 0;

/**
 * @brief calculate bmi088 data in signed form  from LSB into Degree per second
 */
 float gyro_x_in_degree = 0;
 float gyro_y_in_degree = 0;
 float gyro_z_in_degree = 0;

/**
 * @brief Initialize angles
 */  
double yaw = 0.0;
double pitch = 0.0;
double roll = 0.0;

/**
 * @brief Initialize previous angles
 */
double prev_yaw = 0.0;
double prev_pitch = 0.0;
double prev_roll = 0.0;  

/**
 * @brief Initialize the previous timestamp
 */
int64_t previous_timestamp = 0;

/* Initialize other variables */
double pitch_acc = 0.0;
double roll_acc = 0.0;
double gyro_x_rad = 0.0; 
double gyro_y_rad = 0.0;  
double gyro_z_rad = 0.0; 

/**
 * @brief bmi088 IMU Register Addresses
 */
typedef enum
{
    ACC_CHIP_ID    = 0X00,
    ACC_ERR_REG    = 0X02,
    ACC_STATUS     = 0X03,
    ACC_X_LSB      = 0X12,
    ACC_X_MSB      = 0X13,
    ACC_Y_LSB      = 0X14,
    ACC_Y_MSB      = 0X15,
    ACC_Z_LSB      = 0X16,
    ACC_Z_MSB      = 0X17,
    ACC_CONF       = 0X40,
    ACC_RANGE      = 0X41,
    ACC_SELF_TEST  = 0X6D,
    ACC_PWR_CONF   = 0X7C,
    ACC_PWR_CTRL   = 0X7D,
    ACC_SOFT_REST  = 0X7E,
    GYRO_CHIP_ID   = 0X00,
    GYRO_X_LSB     = 0X02,
    GYRO_X_MSB     = 0X03,
    GYRO_Y_LSB     = 0X04,
    GYRO_Y_MSB     = 0X05,
    GYRO_Z_LSB     = 0X06,
    GYRO_Z_MSB     = 0X07,
    GYRO_RANGE     = 0X0F,
    GYRO_BANDWIDTH = 0X10,
    GYRO_LPM1      = 0X11,
    GYRO_SOFT_REST = 0X14,
    GYRO_SELF_TEST = 0X3C,

}mpu_register_address;

uint8_t BMI088_IMU::accel_range = 0; 
uint8_t BMI088_IMU::gyro_range = 0;

void BMI088_IMU::IMU_INIT(){
    uint8_t data[2];

    /* Read the bmi088 GYRO_CHIP_ID REGISTER, on power up the register should have the value 0x0F */
    bmi088_gyro_read(GYRO_CHIP_ID, data, 1);
    printf("GYRO_CHIP_ID_REGISTER_VALUE = %X", data[0]);

    /* Read the bmi088 ACC_PWR_CTRL REGISTER to check power on reset */
    uint8_t check_por[2];
    bmi088_accel_read(ACC_PWR_CTRL, check_por, 1);
    // msleep(1);

    /* Enable accel module by writing 0x04 to power control register */
    uint8_t acc_pwr_ctr = 0x04;
    acc_pwr_ctr |= check_por[0];

    bmi088_accel_write_byte(ACC_PWR_CTRL, acc_pwr_ctr);
    // msleep(50);
    bmi088_accel_read(ACC_PWR_CTRL, check_por, 1);
    
    if (check_por[0] == acc_pwr_ctr)
    {
        printf("Accelerometer configured successfully");
    }
    else printf("Accelerometer not configured ");

    // Check current dynamic ranges
    read_accel_range();
    read_gyro_range();

    // Set desired dynamic ranges (Modify these values as per your requirements)
    set_accel_range(0x03); // Example: Set accelerometer range to ±24g
    set_gyro_range(0x00);  // Example: Set gyroscope range to ±2000 °/s

    // Verify if ranges are set correctly
    read_accel_range();
    read_gyro_range();
}

/**
* @brief Read a sequence of bytes from a bmi088 accel sensor registers
*/
Status BMI088_IMU::bmi088_accel_read(uint8_t reg_addr, uint8_t *data, size_t len){

    Status trans = i2c_master_transmit(I2C0, bmi088_ACCEL_ADDRESS, &reg_addr, 1, 1000 );
    Status rcv = i2c_master_receive(I2C0, bmi088_ACCEL_ADDRESS, data, (uint16_t)len, 1000 );
    return trans;
}

/**
* @brief Read a sequence of bytes from a bmi088 gyro sensor registers
*/
Status BMI088_IMU::bmi088_gyro_read(uint8_t reg_addr, uint8_t *data, size_t len){
    Status trans = i2c_master_transmit(I2C0, bmi088_GYRO_ADDRESS, &reg_addr, 1, 1000 );
    Status rcv = i2c_master_receive(I2C0, bmi088_GYRO_ADDRESS, data, (uint16_t)len, 1000 );
    return trans;
}

/**
* @brief Write a byte to a bmi088 accel sensor register
*/
Status BMI088_IMU::bmi088_accel_write_byte(uint8_t reg_addr, uint8_t data){
    uint8_t write_buf[2] = {reg_addr, data};

    Status trans = i2c_master_transmit(I2C0, bmi088_ACCEL_ADDRESS, write_buf, sizeof(write_buf), 1000 );
    return trans;
}

/**
* @brief Write a byte to a bmi088 gyro sensor register
*/
Status BMI088_IMU::bmi088_gyro_write_byte(uint8_t reg_addr, uint8_t data){
    uint8_t write_buf[2] = {reg_addr, data};
    Status trans = i2c_master_transmit(I2C0, bmi088_GYRO_ADDRESS, write_buf, sizeof(write_buf), 1000 );
    return trans;
}

/**
* @brief i2c master initialization
*/
Status BMI088_IMU::i2c_master_init(void){
    I2C_InitType I2C_init_config;
    I2C_init_config.clock = 40000;
    i2c_init(I2C0, &I2C_init_config);
}

/**
 * @brief Read and report the current accelerometer dynamic range
 */
void BMI088_IMU::read_accel_range() {
    uint8_t range;
    bmi088_accel_read(ACC_RANGE, &range, 1); // ACC_RANGE is the register 0x41

    accel_range = range; // Update member variable

    switch (range) {
        case 0x00:
            printf("Current accelerometer range: ±3g");
            break;
        case 0x01:
            printf("Current accelerometer range: ±6g");
            break;
        case 0x02:
            printf("Current accelerometer range: ±12g");
            break;
        case 0x03:
            printf("Current accelerometer range: ±24g");
            break;
        default:
            printf("Unknown accelerometer range");
    }
}

/**
 * @brief Set the accelerometer dynamic range
 * @param range The dynamic range to set (0x00 for ±3g, 0x01 for ±6g, etc.)
 */
void BMI088_IMU::set_accel_range(uint8_t range) {
    bmi088_accel_write_byte(ACC_RANGE, range); // ACC_RANGE is the register 0x41
    accel_range = range; // Update member variable
    printf("Accelerometer range set");
}

/**
 * @brief Read and report the current gyroscope dynamic range
 */
void BMI088_IMU::read_gyro_range() {
    uint8_t range;
    bmi088_gyro_read(GYRO_RANGE, &range, 1); // GYRO_RANGE is the register 0x0F
    gyro_range = range; // Update member variable

    switch (range) {
        case 0x00:
            printf("Current gyroscope range: ±2000 °/s");
            break;
        case 0x01:
            printf("Current gyroscope range: ±1000 °/s");
            break;
        case 0x02:
            printf("Current gyroscope range: ±500 °/s");
            break;
        case 0x03:
            printf("Current gyroscope range: ±250 °/s");
            break;
        case 0x04:
            printf("Current gyroscope range: ±125 °/s");
            break;
        default:
            printf("Unknown gyroscope range");
    }
}

/**
 * @brief Set the gyroscope dynamic range
 * @param range The dynamic range to set (0x00 for ±2000 °/s, etc.)
 */
void BMI088_IMU::set_gyro_range(uint8_t range) {
    bmi088_gyro_write_byte(GYRO_RANGE, range); // GYRO_RANGE is the register 0x0F
    gyro_range = range; // Update member variable
    printf("Gyroscope range set");
}


float BMI088_IMU::lsb_to_mps2(int16_t val, uint8_t bit_width) {
    float g_range;
    switch (accel_range) {
        case 0x00: g_range = 3.0f; break;  // ±3g
        case 0x01: g_range = 6.0f; break;  // ±6g
        case 0x02: g_range = 12.0f; break; // ±12g
        case 0x03: g_range = 24.0f; break; // ±24g
        default: g_range = 6.0f;           // Default to ±6g
    }

    float half_scale = (1 << (bit_width - 1));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

float BMI088_IMU::lsb_to_dps(int16_t val, uint8_t bit_width) {
    float dps_range;
    switch (gyro_range) {
        case 0x00: dps_range = 2000.0f; break; // ±2000 °/s
        case 0x01: dps_range = 1000.0f; break; // ±1000 °/s
        case 0x02: dps_range = 500.0f; break;  // ±500 °/s
        case 0x03: dps_range = 250.0f; break;  // ±250 °/s
        case 0x04: dps_range = 125.0f; break;  // ±125 °/s
        default: dps_range = 2000.0f;         // Default to ±2000 °/s
    }

    float half_scale = (1 << (bit_width - 1));
    return (dps_range / half_scale) * val;
}

/*!
 * @brief This function reads raw accel_x LSB data and converts to degree per second
 */
double BMI088_IMU::accel_read_rawX(){
    uint8_t lsb, msb;
    uint16_t msblsb;
    bmi088_accel_read(ACC_X_LSB, accel_x, 2);
    lsb = accel_x[0];
    msb = accel_x[1];
    msblsb = (msb << 8) | lsb;
    float x_original_int = ((int16_t) msblsb); /* Data in X axis */
    float x = -1 * lsb_to_mps2(x_original_int, 16);
    return x;
}

/*!
 * @brief This function reads raw accel_y LSB data and converts to degree per second
 */
double BMI088_IMU::accel_read_rawY(){
    uint8_t lsb, msb;
    uint16_t msblsb;
    bmi088_accel_read(ACC_Y_LSB, accel_y, 2);
    lsb = accel_y[0];
    msb = accel_y[1];
    msblsb = (msb << 8) | lsb;
    float y_original_int = ((int16_t) msblsb); /* Data in Y axis */
    float y = -1 * lsb_to_mps2(y_original_int, 16);
    return y;
}

/*!
 * @brief This function reads raw accel_z LSB data and converts to degree per second
 */
double BMI088_IMU::accel_read_rawZ(){
    uint8_t lsb, msb;
    uint16_t msblsb;
    bmi088_accel_read(ACC_Z_LSB, accel_z, 2);
    lsb = accel_z[0];
    msb = accel_z[1];
    msblsb = (msb << 8) | lsb;
    float z_original_int = ((int16_t) msblsb); /* Data in Z axis */
    float z = lsb_to_mps2(z_original_int, 16);
    return z;
}

/*!
 * @brief This function reads raw gyro_x LSB data and converts to degree per second
 */
double BMI088_IMU::gyro_read_rawX(){
    uint8_t lsb, msb;
    uint16_t msblsb;
    bmi088_gyro_read(GYRO_X_LSB, gyro_x, 2);
    lsb = gyro_x[0];
    msb = gyro_x[1];
    msblsb = (msb << 8) | lsb;
    float x_original_int = ((int16_t) msblsb); /* Data in X axis */
    float x = -1 * lsb_to_dps(x_original_int, 16);
    return x;
}

/*!
 * @brief This function reads raw gyro_y LSB data and converts to degree per second
 */
double BMI088_IMU::gyro_read_rawY(){
    uint8_t lsb, msb;
    uint16_t msblsb;
    bmi088_gyro_read(GYRO_Y_LSB, gyro_y, 2);
    lsb = gyro_y[0];
    msb = gyro_y[1];
    msblsb = (msb << 8) | lsb;
    float y_original_int = ((int16_t) msblsb); /* Data in Y axis */
    float y = -1 * lsb_to_dps(y_original_int, 16);
    return y;
}

/*!
 * @brief This function reads raw gyro_z LSB data and converts to degree per second
 */
double BMI088_IMU::gyro_read_rawZ(){
    uint8_t lsb, msb;
    uint16_t msblsb;
    bmi088_gyro_read(GYRO_Z_LSB, gyro_z, 2);
    lsb = gyro_z[0];
    msb = gyro_z[1];
    msblsb = (msb << 8) | lsb;
    float z_original_int = ((int16_t) msblsb); /* Data in Z axis */
    float z = lsb_to_dps(z_original_int, 16);
    return z;
}

/*!
 * @brief This function performs a burst read of the accelerometer and gyroscope
 *        data from the BMI088 sensor and converts the raw data to physical units.
 * @param[out] accelX Pointer to store the scaled accelerometer X value.
 * @param[out] accelY Pointer to store the scaled accelerometer Y value.
 * @param[out] accelZ Pointer to store the scaled accelerometer Z value.
 * @param[out] gyroX Pointer to store the scaled gyroscope X value.
 * @param[out] gyroY Pointer to store the scaled gyroscope Y value.
 * @param[out] gyroZ Pointer to store the scaled gyroscope Z value.
 */
void BMI088_IMU::read_burst(float* accelX, float* accelY, float* accelZ,
                            float* gyroX, float* gyroY, float* gyroZ) {
    // Buffers to hold raw data
    uint8_t accel_full[6];
    uint8_t gyro_full[6];

    // Read 6 bytes from the accelerometer
    bmi088_accel_read(ACC_X_LSB, accel_full, 6);
    *accelX = -1 * lsb_to_mps2((int16_t)((accel_full[1] << 8) | accel_full[0]), 16);
    *accelY = -1 * lsb_to_mps2((int16_t)((accel_full[3] << 8) | accel_full[2]), 16);
    *accelZ = lsb_to_mps2((int16_t)((accel_full[5] << 8) | accel_full[4]), 16);

    bmi088_gyro_read(GYRO_X_LSB, gyro_full, 6);
    // Convert raw gyroscope data
    *gyroX = -1 * lsb_to_dps((int16_t)((gyro_full[1] << 8) | gyro_full[0]), 16);
    *gyroY = -1 * lsb_to_dps((int16_t)((gyro_full[3] << 8) | gyro_full[2]), 16);
    *gyroZ = -1 * lsb_to_dps((int16_t)((gyro_full[5] << 8) | gyro_full[4]), 16);
}


double BMI088_IMU::angle_read_pitch(){
    double x_Buff = accel_read_rawX();
    double y_Buff = accel_read_rawY();
    double z_Buff = accel_read_rawZ();
    double pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
    return pitch;
}

double BMI088_IMU::angle_read_roll(){
    double y_Buff = accel_read_rawY();
    double z_Buff = accel_read_rawZ();
    double roll = atan2(y_Buff , z_Buff) * 57.3;
    return roll;
}

double BMI088_IMU::angle_read_yaw(){
    return 0;
}

// int BMI088_IMU::msleep(useconds_t msec) {
//   #ifdef CLINT_BASE
//     uint64_t target_tick = clint_get_time((CLINT_Type *)CLINT_BASE) + ((msec * 50000) / 1000);
//     while (clint_get_time((CLINT_Type *)CLINT_BASE) < target_tick) {
//       asm volatile("nop");
//     }
//   #else
//     #warning "No CLINT peripheral found. Delay function is not available."
//   #endif

//   return 0;
// }