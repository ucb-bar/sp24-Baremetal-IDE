#ifndef IMU
#define IMU

#include <stdint.h>
#include <math.h>
#include "chip_config.h"

#define PITCH (uint8_t) 0
#define ROLL (uint8_t) 1
#define YAW (uint8_t) 2

class BMI088_IMU {
    public:
        static void IMU_INIT();
        /**
         * @brief Read a sequence of bytes from a bmi088 accel sensor registers
         */
        static Status bmi088_accel_read(uint8_t reg_addr, uint8_t *data, size_t len);

        /**
         * @brief Read a sequence of bytes from a bmi088 gyro sensor registers
         */
        static Status bmi088_gyro_read(uint8_t reg_addr, uint8_t *data, size_t len);

        /**
         * @brief Write a byte to a bmi088 accel sensor register
         */
        static Status bmi088_accel_write_byte(uint8_t reg_addr, uint8_t data);

        /**
         * @brief Write a byte to a bmi088 gyro sensor register
        */
        static Status bmi088_gyro_write_byte(uint8_t reg_addr, uint8_t data);

        /**
         * @brief i2c master initialization
         */
        static Status i2c_master_init(void);

        /*!
        * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
        * range 2G, 4G, 8G or 16G.
        */
        static float lsb_to_mps2(int16_t val, uint8_t bit_width);

        /*!
        * @brief This function converts lsb to degree per second for 16 bit gyro at
        * range 125, 250, 500, 1000 or 2000dps.
        */
        static float lsb_to_dps(int16_t val, uint8_t bit_width);

        /*!
        * @brief This function reads raw accel_x LSB data and converts to degree per second
        */
        static double accel_read_rawX();

        /*!
        * @brief This function reads raw accel_y LSB data and converts to degree per second
        */
        static double accel_read_rawY();

        /*!
        * @brief This function reads raw accel_z LSB data and converts to degree per second
        */
        static double accel_read_rawZ();

        /*!
        * @brief This function reads raw gyro_x LSB data and converts to degree per second
        */
        static double gyro_read_rawX();

        /*!
        * @brief This function reads raw gyro_y LSB data and converts to degree per second
        */
        static double gyro_read_rawY();

        /*!
        * @brief This function reads raw gyro_z LSB data and converts to degree per second
        */
        static double gyro_read_rawZ();

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
        void read_burst(float* accelX, float* accelY, float* accelZ, float* gyroX, float* gyroY, float* gyroZ);


        static double angle_read_pitch();

        static double angle_read_roll();

        static double angle_read_yaw();

        static double linearInterpolate(double input, double input_start, double input_end,
                                            double output_start, double output_end);

        static void read_accel_range();
        static void set_accel_range(uint8_t range);
        static void read_gyro_range();
        static void set_gyro_range(uint8_t range);

        // static int msleep(useconds_t msec);

    private:
        static uint8_t accel_range; // To store the current accelerometer range
        static uint8_t gyro_range;  // To store the current gyroscope range

};

#endif //IMU