MPU6050 Library for ESP32
This library provides functions to interface with the MPU6050 sensor using an ESP32 microcontroller. The MPU6050 sensor is a six-axis MotionTracking device that combines a 3-axis gyroscope and a 3-axis accelerometer. This library supports reading accelerometer and gyroscope data, as well as calculating orientation using quaternions and Euler angles.

Table of Contents
Installation
Usage
Initialization
Reading Sensor Data
Calculating Orientation
Functions
Example
License
Installation
Clone the repository to your ESP32 project directory:

bash
Copy code
git clone https://github.com/yourusername/MPU6050-ESP32.git
Include the library in your project by adding the following lines to your CMakeLists.txt or component.mk file:

cmake
Copy code
set(EXTRA_COMPONENT_DIRS "/path/to/MPU6050-ESP32")
Usage
Initialization
To initialize the MPU6050 sensor, call the mpu6050_init() function. This function configures the I2C interface and wakes up the sensor.

c
Copy code
#include "mpu6050.h"

void app_main(void) {
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "MPU6050 initialization failed");
        return;
    }
}
Reading Sensor Data
You can read accelerometer and gyroscope data using the mpu6050_read_accel() and mpu6050_read_gyro() functions, respectively.

c
Copy code
float ax, ay, az, gx, gy, gz;

esp_err_t ret = mpu6050_read_accel(&ax, &ay, &az);
if (ret == ESP_OK) {
    ESP_LOGI("APP", "Acceleration X: %.2f, Y: %.2f, Z: %.2f", ax, ay, az);
} else {
    ESP_LOGE("APP", "Failed to read acceleration data");
}

ret = mpu6050_read_gyro(&gx, &gy, &gz);
if (ret == ESP_OK) {
    ESP_LOGI("APP", "Gyro X: %.2f, Y: %.2f, Z: %.2f", gx, gy, gz);
} else {
    ESP_LOGE("APP", "Failed to read gyroscope data");
}
Calculating Orientation
The library provides functions to calculate roll and pitch angles from accelerometer data and to convert quaternions to Euler angles.

c
Copy code
float roll = calculate_roll(ax, ay, az);
float pitch = calculate_pitch(ax, ay, az);

Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};
update_quaternion(gx, gy, gz, sample_rate, &q);

float yaw;
quaternion_to_euler(q, &roll, &pitch, &yaw);
ESP_LOGI("APP", "Quaternion to Euler - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
Functions
Initialization
esp_err_t mpu6050_init(void);
Reading Sensor Data
esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az);
esp_err_t mpu6050_read_gyro(float *gx, float *gy, float *gz);
Calculating Orientation
float calculate_roll(float ax, float ay, float az);
float calculate_pitch(float ax, float ay, float az);
void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw);
void update_quaternion(float gx, float gy, float gz, float dt, Quaternion *q);
Example
c
Copy code
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
License
This project is licensed under the MIT License. See the LICENSE file for details.
