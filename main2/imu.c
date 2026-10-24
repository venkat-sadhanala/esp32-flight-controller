
#include "imu.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define I2C_SDA_GPIO       21
#define I2C_SCL_GPIO       22
#define MPU9250_ADDR       0x68

#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_CONFIG   0x1C
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_XOUT_H   0x3B

static const char *TAG = "IMU";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t mpu_handle;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};

    return i2c_master_transmit(
        mpu_handle, data, sizeof(data), 1000
    );
}

static esp_err_t read_regs(
    uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(
        mpu_handle, &reg, 1, data, length, 1000
    );
}

esp_err_t imu_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(
        &bus_config, &bus_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDR,
        .scl_speed_hz = 400000,
    };

    err = i2c_master_bus_add_device(
        bus_handle, &dev_config, &mpu_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU device setup failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    uint8_t who_am_i = 0;
    err = read_regs(REG_WHO_AM_I, &who_am_i, 1);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed");
        return err;
    }

    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who_am_i);

    // MPU9250 normally returns 0x71.
    // Some compatible devices return 0x73.
    if (who_am_i != 0x70 &&
    who_am_i != 0x71 &&
    who_am_i != 0x73) {
    ESP_LOGE(TAG, "Unexpected sensor identity");
    return ESP_ERR_NOT_FOUND;
}
    // Wake the sensor.
    err = write_reg(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(100));

    // Accelerometer: +/- 2 g.
    err = write_reg(REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) return err;

    // Gyroscope: +/- 250 degrees/second.
    err = write_reg(REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "MPU9250 initialized");
    return ESP_OK;
}

esp_err_t imu_read(imu_raw_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[14];

    esp_err_t err = read_regs(
        REG_ACCEL_XOUT_H, buffer, sizeof(buffer)
    );

    if (err != ESP_OK) {
        return err;
    }

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Bytes 6-7 contain temperature; skip them.
    data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}
