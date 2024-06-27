#include <stdio.h>
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "MPU6050 initialization failed");
        return;
    }
    
    Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // Initial quaternion
    float ax, ay, az, gx, gy, gz;
    float roll, pitch, yaw;
    const float sample_rate = 0.01f; // Sample rate in seconds (100 Hz)

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
       
            // Update quaternion
            update_quaternion(gx, gy, gz, sample_rate, &q);
            
            // Convert quaternion to Euler angles
            quaternion_to_euler(q, &roll, &pitch, &yaw);
            ESP_LOGI("APP", "Quaternion to Euler - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
        } else {
            ESP_LOGE("APP", "Failed to read gyroscope data");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
        printf("\n");
    }
}
