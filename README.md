# **MPU6050 Library for ESP32**
This library provides functions to interface with the MPU6050 sensor using an ESP32 microcontroller. The MPU6050 sensor is a six-axis MotionTracking device that combines a 3-axis gyroscope and a 3-axis accelerometer. This library supports reading accelerometer and gyroscope data, as well as calculating orientation using quaternions and Euler angles.

## Table of Contents
1. Installation
2. Hardware Setup
3. Usage
 - Initialization
 - Reading Sensor Data
 - Calculating Orientation
 - Functions
 - Example
4. Demo Video



## 1. Installation
Clone the repository to your ESP32 project directory:
```
git clone https://github.com/Zeshan7866/MPU6050-for-ESP32.git
```
Include the library in your project by adding the following lines to your CMakeLists.txt or component.mk file:
```
set(EXTRA_COMPONENT_DIRS "/path/to/MPU6050-ESP32")
```
## 2. Hardware Setup
### 1. Connect the MPU6050 to the ESP32:
 - MPU6050 VCC to ESP32 3.3V
 - MPU6050 GND to ESP32 GND
 - MPU6050 SCL to ESP32 GPIO 22
 - MPU6050 SDA to ESP32 GPIO 21

### 2. Diagram

![Screenshot of a comment on a GitHub issue showing an image, added in the Markdown, of an Octocat smiling and raising a tentacle.](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2020/12/MPU6050_ESP32_Wiring-Schematic-Diagram.png?w=726&quality=100&strip=all&ssl=1))

### 3.  I2C Address
The default I2C address of the MPU6050 is `0x68`.
## 3. Usage
### 1. Initialization
To initialize the MPU6050 sensor, call the mpu6050_init() function. This function configures the I2C interface and wakes up the sensor.
```
#include "mpu6050.h"

void app_main(void) {
    esp_err_t ret = mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "MPU6050 initialization failed");
        return;
    }
}
```
### 2. Reading Sensor Data
You can read accelerometer and gyroscope data using the `mpu6050_read_accel()` and `mpu6050_read_gyro()` functions, respectively.
```
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
```
### 3. Calculating Orientation
The library provides functions to calculate roll and pitch angles from accelerometer data and to convert quaternions to Euler angles.
```
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

```
### 4. Example
```
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
```

## 4. Demo Video

[Watch the video](https://www.loom.com/share/be3c0a4d6e204881b2f0ee6edb626746?sid=55102f28-1c8d-4558-9639-90e6e8658ce2)

