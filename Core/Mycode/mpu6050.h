//
// Created by Administrator on 2026/4/7.
//

#ifndef AIRMOUSE_MPU6050_H
#define AIRMOUSE_MPU6050_H

#include "i2c.h"

#define MPU6050_ADDR          (0x68 << 1)   // AD0接地时的地址
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_PWR_MGMT_2    0x6C
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_GYRO_XOUT_H   0x43    // 陀螺仪X轴高8位寄存器
#define MPU6050_ACCEL_XOUT_H   0x3B    // 加速度计X轴高8位寄存器

void MPU6050_WriteReg(uint8_t address,uint8_t data);

void MPU6050_Init(void);
void MPU6050_GetGyroDPS(float *wx, float *wy, float *wz);
void MPU6050_Calibrate(void);
void MPU6050_GetAccel(float *ax, float *ay, float *az);

#endif //AIRMOUSE_MPU6050_H