#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

#define DEG_TO_RAD (M_PI / 180.0)


static const char *TAG = "MPU6050";

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, buffer, sizeof(buffer), 1000 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init() {
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return ret;
    }

    // Wake up the MPU6050
    ret = mpu6050_write_register(0x6B, 0x00); // PWR_MGMT_1 register
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }

    // Set accelerometer full-scale range to ±4g
    ret = mpu6050_write_register(0x1C, 0x08); // ACCEL_CONFIG register
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer full-scale range");
        return ret;
    }

    // Set gyroscope full-scale range to ±500°/s
    ret = mpu6050_write_register(0x1B, 0x08); // GYRO_CONFIG register
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope full-scale range");
    }

    return ret;
}

static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az) {
    uint8_t data[6];
    esp_err_t ret = mpu6050_read(0x3B, data, sizeof(data)); // ACCEL_XOUT_H register
    if (ret != ESP_OK) {
        return ret;
    }

    // Assuming ±4g full-scale range (sensitivity = 8192 LSB/g)
    *ax = ((int16_t)(data[0] << 8 | data[1])) / 8192.0;
    *ay = ((int16_t)(data[2] << 8 | data[3])) / 8192.0;
    *az = ((int16_t)(data[4] << 8 | data[5])) / 8192.0;

    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(float *gx, float *gy, float *gz) {
    uint8_t data[6];
    esp_err_t ret = mpu6050_read(0x43, data, sizeof(data)); // GYRO_XOUT_H register
    if (ret != ESP_OK) {
        return ret;
    }

    // Assuming ±500°/s full-scale range (sensitivity = 65.5 LSB/(°/s))
    *gx = ((int16_t)(data[0] << 8 | data[1])) / 65.5;
    *gy = ((int16_t)(data[2] << 8 | data[3])) / 65.5;
    *gz = ((int16_t)(data[4] << 8 | data[5])) / 65.5;

    return ESP_OK;
}

float calculate_roll(float ax, float ay, float az) {
    return atan2(ay, az) * 180 / M_PI;
}

float calculate_pitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
}


void update_quaternion(float gx, float gy, float gz, float dt, Quaternion *q) {
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    float q1 = q->w;
    float q2 = q->x;
    float q3 = q->y;
    float q4 = q->z;

    float q_dot_w = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
    float q_dot_x = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
    float q_dot_y = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
    float q_dot_z = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

    q->w += q_dot_w * dt;
    q->x += q_dot_x * dt;
    q->y += q_dot_y * dt;
    q->z += q_dot_z * dt;

    // Normalize quaternion
    float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180 / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp) * 180 / M_PI; // use 90 degrees if out of range
    else
        *pitch = asin(sinp) * 180 / M_PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    *yaw = atan2(siny_cosp, cosy_cosp) * 180 / M_PI;
}
