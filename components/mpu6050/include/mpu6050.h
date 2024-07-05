//Inclusion Guards : For preventing multiple inclusions of this header file, which can lead to redefinition errors.
#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h" // esp error handling library
#include "driver/i2c.h"
// ensures library can be used in both c and cpp
#ifdef __cplusplus 
extern "C" {
#endif

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// Quaternion Structure
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

// Function prototypes
esp_err_t mpu6050_init(int scl_io, int sda_io, int i2c_master_freq, i2c_port_t i2c_master_port);
esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az);
esp_err_t mpu6050_read_gyro(float *gx, float *gy, float *gz);
float calculate_roll(float ax, float ay, float az);
float calculate_pitch(float ax, float ay, float az);
void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw);
void update_quaternion(float gx, float gy, float gz, float dt, Quaternion *q); 
esp_err_t mpu6050_init_dmp(void);
esp_err_t mpu6050_get_quaternion(Quaternion *q);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H
