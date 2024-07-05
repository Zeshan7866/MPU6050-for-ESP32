#include <stdio.h>
#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "MPU6050";

void app_main(void) {
    esp_err_t ret;

    // Initialize MPU6050 with I2C parameters
    ret = mpu6050_init(-1, -1, -1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return;
    }

    // Initialize DMP
    ret = mpu6050_init_dmp();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DMP");
        return;
    }

    Quaternion q;
    float roll, pitch, yaw;

    while (1) {
        // Get quaternion from DMP
        ret = mpu6050_get_quaternion(&q);
        if (ret == ESP_OK) {
            // Convert quaternion to Euler angles
            quaternion_to_euler(q, &roll, &pitch, &yaw);

            ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
        } else {
            ESP_LOGE(TAG, "Failed to get quaternion data");
        }

        // Delay to avoid spamming the log
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
