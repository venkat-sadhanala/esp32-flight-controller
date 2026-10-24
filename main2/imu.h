
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_raw_data_t;

esp_err_t imu_init(void);
esp_err_t imu_read(imu_raw_data_t *data);

#endif
