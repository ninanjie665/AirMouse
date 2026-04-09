//
// Created by Administrator on 2026/4/9.
//

#ifndef AIRMOUSE_MOUSE_H
#define AIRMOUSE_MOUSE_H
#include "mpu6050.h"
#include "mahony.h"

static uint8_t is_key_pressed(GPIO_TypeDef* port, uint16_t pin);
void Gyro_To_MouseMove();
void Angal_To_MouseMove();
#endif //AIRMOUSE_MOUSE_H