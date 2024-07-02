/**
 * @file mpu6050_driver.c
 * @brief MPU6050 sensor driver for ESP32 using I2C.
 */

#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

// I2C configuration
#define I2C_MASTER_SCL_IO 22        /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number for I2C master data */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master port number */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define DEG_TO_RAD (M_PI / 180.0)

static const char *TAG = "MPU6050";

// Function prototypes
static esp_err_t i2c_master_init(int scl_io, int sda_io, int i2c_master_freq, i2c_port_t i2c_master_port);
static esp_err_t mpu6050_write_register(uint8_t reg, uint8_t value);
static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len);

static esp_err_t mpu6050_write_memory_block(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
static esp_err_t mpu6050_set_memory_bank(uint8_t bank, bool prefetchEnabled, bool userBank);
static esp_err_t mpu6050_set_memory_start_address(uint8_t address);
static esp_err_t mpu6050_write_dmp_configuration_set(const uint8_t *data, uint16_t dataSize);



/**
 * @brief Initialize I2C master interface.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t i2c_master_init(int scl_io, int sda_io, int i2c_master_freq, i2c_port_t i2c_master_port) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_io == -1 ? I2C_MASTER_SDA_IO : sda_io,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_io == -1 ? I2C_MASTER_SCL_IO : scl_io,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_master_freq == -1 ? I2C_MASTER_FREQ_HZ : i2c_master_freq,
    };
    esp_err_t err = i2c_param_config(i2c_master_port == -1 ? I2C_MASTER_NUM : i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/**
 * @brief Write a value to a MPU6050 register.
 * @param reg Register address.
 * @param value Value to write.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t mpu6050_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, buffer, sizeof(buffer), 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Initialize the MPU6050 sensor.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t mpu6050_init(int scl_io , int sda_io, int i2c_master_freq, i2c_port_t i2c_master_port) {
    esp_err_t ret = i2c_master_init(scl_io, sda_io, i2c_master_freq, i2c_master_port);
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

/**
 * @brief Read data from the MPU6050.
 * @param reg Register address.
 * @param data Buffer to store the read data.
 * @param len Length of data to read.
 * @return ESP_OK on success, or an error code on failure.
 */
static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Read accelerometer data from the MPU6050.
 * @param ax Pointer to store X-axis acceleration.
 * @param ay Pointer to store Y-axis acceleration.
 * @param az Pointer to store Z-axis acceleration.
 * @return ESP_OK on success, or an error code on failure.
 */
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

/**
 * @brief Read gyroscope data from the MPU6050.
 * @param gx Pointer to store X-axis angular velocity.
 * @param gy Pointer to store Y-axis angular velocity.
 * @param gz Pointer to store Z-axis angular velocity.
 * @return ESP_OK on success, or an error code on failure.
 */
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

/**
 * @brief Calculate roll angle from accelerometer data.
 * @param ax X-axis acceleration.
 * @param ay Y-axis acceleration.
 * @param az Z-axis acceleration.
 * @return Roll angle in degrees.
 */
float calculate_roll(float ax, float ay, float az) {
    return atan2(ay, sqrt(ax*ax + az*az)) * 180 / M_PI;
}

/**
 * @brief Calculate pitch angle from accelerometer data.
 * @param ax X-axis acceleration.
 * @param ay Y-axis acceleration.
 * @param az Z-axis acceleration.
 * @return Pitch angle in degrees.
 */
float calculate_pitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
}

/**
 * @brief Update quaternion values based on gyroscope data.
 * @param gx X-axis angular velocity.
 * @param gy Y-axis angular velocity.
 * @param gz Z-axis angular velocity.
 * @param dt Time interval.
 * @param q Pointer to quaternion structure.
 */
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

/**
 * @brief Convert quaternion to Euler angles.
 * @param q Quaternion structure.
 * @param roll Pointer to store roll angle.
 * @param pitch Pointer to store pitch angle.
 * @param yaw Pointer to store yaw angle.
 */
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

static esp_err_t mpu6050_write_memory_block(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
    esp_err_t ret = mpu6050_set_memory_bank(bank, false, false);
    if (ret != ESP_OK) return ret;
    ret = mpu6050_set_memory_start_address(address);
    if (ret != ESP_OK) return ret;

    const uint8_t chunkSize = 16;
    for (uint16_t i = 0; i < dataSize; i += chunkSize) {
        uint16_t bytesToWrite = (dataSize - i < chunkSize) ? dataSize - i : chunkSize;
        ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data + i, bytesToWrite, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static esp_err_t mpu6050_set_memory_bank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    return mpu6050_write_register(0x6D, bank | (prefetchEnabled << 6) | (userBank << 5));
}

static esp_err_t mpu6050_set_memory_start_address(uint8_t address) {
    return mpu6050_write_register(0x6E, address);
}

static esp_err_t mpu6050_write_dmp_configuration_set(const uint8_t *data, uint16_t dataSize) {
    return mpu6050_write_memory_block(data, dataSize, 0x00, 0x00);
}

esp_err_t mpu6050_init_dmp(void) {
    // Load DMP firmware
    // Note: DMP firmware data must be defined as an array of uint8_t (not provided here)
    extern const uint8_t dmp_firmware[];
    extern const uint16_t dmp_firmware_size;

    esp_err_t ret = mpu6050_write_dmp_configuration_set(dmp_firmware, dmp_firmware_size);
    if (ret != ESP_OK) return ret;

    // Enable DMP
    ret = mpu6050_write_register(0x6A, 0xC0); // USER_CTRL register
    if (ret != ESP_OK) return ret;
    ret = mpu6050_write_register(0x38, 0x02); // INT_ENABLE register
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t mpu6050_get_quaternion(Quaternion *q) {
    uint8_t data[16];
    esp_err_t ret = mpu6050_read(0x3B, data, sizeof(data)); // DMP packet
    if (ret != ESP_OK) return ret;

    int32_t q1 = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    int32_t q2 = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    int32_t q3 = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
    int32_t q4 = (data[12] << 24) | (data[13] << 16) | (data[14] << 8) | data[15];

    const float scale = 1073741824.0f; // 2^30
    q->w = q1 / scale;
    q->x = q2 / scale;
    q->y = q3 / scale;
    q->z = q4 / scale;

    return ESP_OK;
}

