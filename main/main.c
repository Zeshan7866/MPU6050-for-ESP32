#include <stdio.h>
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22        /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< GPIO number for I2C master data */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C master port number */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

void app_main(void) {
    esp_err_t ret = mpu6050_init(I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO,I2C_MASTER_FREQ_HZ, I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "MPU6050 initialization failed");
        return;
    }
    

    float ax, ay, az, gx, gy, gz;
    float roll, pitch;


    while (1) {
        ret = mpu6050_read_accel(&ax, &ay, &az);
        if (ret == ESP_OK) {
            ESP_LOGI("APP", "Acceleration X: %.2f, Y: %.2f, Z: %.2f", ax, ay, az);
            roll = calculate_roll(ax, ay, az);
            pitch = calculate_pitch(ax, ay, az);
            ESP_LOGI("APP", "Roll: %.2f, Pitch: %.2f", roll, pitch);
        } else {
            ESP_LOGE("APP", "Failed to read acceleration data");
        }

        ret = mpu6050_read_gyro(&gx, &gy, &gz);
        if (ret == ESP_OK) {
            ESP_LOGI("APP", "Gyro X: %.2f, Y: %.2f, Z: %.2f", gx, gy, gz);
       

            
            
        } else {
            ESP_LOGE("APP", "Failed to read gyroscope data");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
        printf("\n");
    }
}
