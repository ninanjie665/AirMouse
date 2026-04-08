#include "mpu6050.h"

#include <stdio.h>
static float gyron_offset_x = 0;
float gyro_sensitivity = 16.4f;


void MPU6050_Calibrate(void) {
    float sum = 0;
    for (int i = 0; i < 200;i++) {
        uint8_t buf[6];
        HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,MPU6050_GYRO_XOUT_H,
            I2C_MEMADD_SIZE_8BIT,buf,6,5);
        int16_t raw_x = (int16_t)((buf[0] << 8) | 8 );
        sum += raw_x / gyro_sensitivity;
        HAL_Delay(5);
    }
    gyron_offset_x = sum / 200;
}

void MPU6050_WriteReg(uint8_t address,uint8_t data) {
    uint8_t buf[2];

    buf[0] = address;
    buf[1] = data;

    HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,buf,2,10);
}


void MPU6050_Init(void) {
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    HAL_Delay(10);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    HAL_Delay(10);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
    HAL_Delay(10);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    HAL_Delay(10);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
    HAL_Delay(10);
}
void MPU6050_GetGyroDPS(float *wx, float *wy, float *wz) {
    uint8_t buf[6];

    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,MPU6050_GYRO_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,buf,6,10);

    int16_t raw_x = (int16_t)((buf[0] << 8) | buf[1]); //x
    int16_t raw_y = (int16_t)((buf[2] << 8) | buf[3]); //y
    int16_t raw_z = (int16_t)((buf[4] << 8) | buf[5]); //z

    float x_dps = raw_x / gyro_sensitivity - gyron_offset_x;
    float y_dps = raw_y / gyro_sensitivity;
    float z_dps = raw_z / gyro_sensitivity;

    static float x_filt = 0, y_filt = 0, z_filt = 0;
    float alpha = 0.2f;

    x_filt = x_dps * alpha + (1 - alpha) * x_filt;
    y_filt = y_dps * alpha + (1 - alpha) * y_filt;
    z_filt = z_dps * alpha + (1 - alpha) * z_filt;

    *wx = x_filt;
    *wy = y_filt;
    *wz = z_filt;
}