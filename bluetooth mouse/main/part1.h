#ifndef PART1_H
#define PART1_H

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"


#define I2C_MASTER_SCL_IO           8      // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           10      // GPIO number for I2C master data
#define I2C_MASTER_NUM              I2C_NUM_0  // I2C port number
#define I2C_MASTER_FREQ_HZ          100000    // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0         // I2C master does not need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0         // I2C master does not need buffer

#define ICM42670_ADDR               0x68      // I2C address of the ICM-42670-P sensor
#define ICM42670_WHO_AM_I           0x75      // Register for device ID


#define ACCEL_XOUT_HIGH             0x0B  // Register for gyroscope X high byte
#define ACCEL_XOUT_LOW              0x0C  // Register for gyroscope X low byte
#define ACCEL_YOUT_HIGH             0x0D  // Register for gyroscope Y high byte
#define ACCEL_YOUT_LOW              0x0E  // Register for gyroscope Y low byte

#define ICM42670_ACCEL_CONFIG0      0x21


static const char* TAG = "InclinationSensor";

// Initializes the I2C master interface
void i2c_master_init(void);
void icm42670_setup();

// Read a byte from a register of the ICM42670 sensor
int16_t read_sensor_data(uint8_t reg_addr_high, uint8_t reg_addr_low);

// More function declarations as needed
// Example:
// int16_t read_gyro_x(void);
// int16_t read_gyro_y(void);

#endif // PART1_H
