#include "part1.h"

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

uint8_t icm42670_read_byte(uint8_t reg_addr) {
    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(5 / portTICK_PERIOD_MS);
    return data;
}

void icm42670_setup() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start I2C communication
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ICM42670_ADDR << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, 0x1F, true);  // pwr
    i2c_master_write_byte(cmd, 0x1F, true);  // reset


    // i2c_master_write_byte(cmd, ICM42670_ACCEL_CONFIG0, true);  // Register address
    // i2c_master_write_byte(cmd, 0x06, true);
    // i2c_master_write_byte(cmd, ICM42670_CLOCK_PLL, true);   // Register value: PLL with X axis gyroscope reference

    // Stop I2C communication
    i2c_master_stop(cmd);

    // Execute the command and wait for it to complete
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);

    // Handle errors
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ICM-42670-P sensor: %s", esp_err_to_name(ret));
    }

    // Delete command handle
    i2c_cmd_link_delete(cmd);

}

int16_t read_sensor_data(uint8_t reg_addr_high, uint8_t reg_addr_low) {
    uint8_t high_byte = icm42670_read_byte(reg_addr_high);
    uint8_t low_byte = icm42670_read_byte(reg_addr_low);
    return (high_byte << 8) | low_byte;
}

void inclination_task(void* arg) {
    int16_t x_accel, y_accel = 0;

    while (1) {
        x_accel = read_sensor_data(ACCEL_XOUT_HIGH, ACCEL_XOUT_LOW);
        
        y_accel = read_sensor_data(ACCEL_YOUT_HIGH, ACCEL_YOUT_LOW);


        // ESP_LOGI(TAG, "Raw ACCEL Data: X=%d, Y=%d", x_accel, y_accel);

        if (x_accel > 500 && y_accel > 500) ESP_LOGI(TAG, "DOWN LEFT");
        else if (x_accel < -500 && y_accel > 500) ESP_LOGI(TAG, "DOWN RIGHT");
        else if (x_accel > 500 && y_accel < -500) ESP_LOGI(TAG, "UP LEFT");
        else if (x_accel < -500 && y_accel < -500) ESP_LOGI(TAG, "UP RIGHT");
        else if (x_accel > 500) ESP_LOGI(TAG, "LEFT");
        else if (x_accel < -500) ESP_LOGI(TAG, "RIGHT");
        else if (y_accel > 500) ESP_LOGI(TAG, "DOWN");
        else if (y_accel < -500) ESP_LOGI(TAG, "UP");

        vTaskDelay(100 / portTICK_PERIOD_MS);

    }


}
