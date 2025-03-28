#include <stdio.h>
#include "DFRobot_LCD.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define I2C_MASTER_SCL_IO           8               /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           10              /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define SHTC3_SENSOR_ADDR           0x70            /*!< Slave address of the SHTC3 sensor */
#define CMD_WAKE                    0x3517
#define CMD_SLEEP                   0xB098
#define CMD_MEASURE                 0x7866          /*!< Command for both T and RH measurement */

DFRobot_LCD lcd(16, 2); // LCD with 16 columns and 2 rows

// static esp_err_t i2c_master_init(void) {
//     i2c_config_t conf = {};  // Zero-initialize the configuration structure
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = I2C_MASTER_SDA_IO;
//     conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
//     conf.scl_io_num = I2C_MASTER_SCL_IO;
//     conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
//     conf.master.clk_speed = I2C_MASTER_FREQ_HZ;  // Set the clock speed for I2C master

//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
// }


static uint8_t calculate_crc(uint8_t data[], int len) {
    uint8_t crc = 0xFF;
    for (int byteIndex = 0; byteIndex < len; ++byteIndex) {
        crc ^= data[byteIndex];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31u;
            else crc = (crc << 1);
        }
    }
    return crc;
}

static float read_temperature(uint8_t data[]) {
    uint16_t rawValue = (data[0] << 8) | data[1];
    return -45 + 175 * ((float) rawValue / 65535);
}

static float read_humidity(uint8_t data[]) {
    uint16_t rawValue = (data[0] << 8) | data[1];
    return 100 * ((float) rawValue / 65535);
}


void read_shtc3_data(void *arg) {
    char display_str[32];
    uint8_t data[6];
    while(1) {
        // Sensor communication and data processing code remains unchanged
        // Wake up the sensor
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, CMD_WAKE >> 8, true);
        i2c_master_write_byte(cmd, CMD_WAKE & 0xFF, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for sensor to wake up

        // Measure and read data
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, CMD_MEASURE >> 8, true);
        i2c_master_write_byte(cmd, CMD_MEASURE & 0xFF, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to allow measurement to complete

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);


        if (calculate_crc(data, 2) == data[2] && calculate_crc(data + 3, 2) == data[5]) {
            float temperature = read_temperature(data);
            float humidity = read_humidity(data + 3);

            snprintf(display_str, sizeof(display_str), "Temp: %.0fC", temperature);
            lcd.setCursor(0, 0);
            lcd.printstr(display_str);

            snprintf(display_str, sizeof(display_str), "Hum : %.0f%%", humidity);
            lcd.setCursor(0, 1);
            lcd.printstr(display_str);
        } else {
            lcd.setCursor(0, 0);
            lcd.printstr("Checksum error!");
        }

        // // Put the sensor back to sleep
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, CMD_SLEEP >> 8, true);
        i2c_master_write_byte(cmd, CMD_SLEEP & 0xFF, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2 seconds before next cycle

    }
}

extern "C" void app_main(void) {
    lcd.init(); // Initialize the LCD
    lcd.setRGB(0, 255, 0); // Set backlight color to green
    // i2c_master_init(); // Initialize I2C communication

    xTaskCreate(read_shtc3_data, "read_shtc3_data", 4096, NULL, 10, NULL);
}
