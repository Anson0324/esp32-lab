#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define TRIGGER_PIN GPIO_NUM_1
#define ECHO_PIN GPIO_NUM_0

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define SHTC3_SENSOR_ADDR           0x70
#define CMD_WAKE                    0x3517
#define CMD_SLEEP                   0xB098
#define CMD_MEASURE                 0x7866

static const char *TAG = "SR04_SHTC3";

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
}

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

static void read_shtc3_data(float *temperature) {
    uint8_t data[6];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
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

    if (calculate_crc(data, 2) == data[2] && calculate_crc(data+3, 2) == data[5]) {
        *temperature = read_temperature(data);
    } else {
        ESP_LOGE(TAG, "Checksum error!");
    }
}

void init_gpio() {
    gpio_reset_pin(TRIGGER_PIN);
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

uint32_t get_pulse_duration() {
    uint32_t start_time = 0;
    uint32_t end_time = 0;

    // Trigger the sensor
    gpio_set_level(TRIGGER_PIN, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(TRIGGER_PIN, 0);

    // Wait for the echo start
    while (gpio_get_level(ECHO_PIN) == 0) {}

    start_time = esp_timer_get_time();

    // Wait for the echo end
    while (gpio_get_level(ECHO_PIN) == 1) {}

    end_time = esp_timer_get_time();

    // Calculate pulse duration
    uint32_t duration = end_time - start_time;
    return duration;
}

float get_distance_cm(float temperature) {
    // uint32_t avg_duration = 0;
    // uint32_t total_duration = 0;
    // const int num_samples = 10;

    // for (int i = 0; i < num_samples; ++i) {
    //     total_duration += get_pulse_duration();
    //     vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay between samples
    // }

    uint32_t avg_duration = get_pulse_duration();

    // avg_duration = total_duration/num_samples;


    // Speed of sound calculation
    float speed_of_sound = 331.4 + (0.6 * temperature); // in m/s
    float speed_of_sound_cm_per_us = speed_of_sound / 10000.0; // convert to cm/us
    float distance = (avg_duration / 2.0) * speed_of_sound_cm_per_us; // in cm
    return distance;
}

void app_main() {
    init_gpio();
    i2c_master_init();

    while (1) {
        float temperature = 0;
        read_shtc3_data(&temperature);

        float distance = get_distance_cm(temperature);
        ESP_LOGI(TAG, "Distance: %.1f cm at %.0fC", distance, temperature);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
