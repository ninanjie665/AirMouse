//
// Created by Administrator on 2026/4/9.
//

#ifndef AIRMOUSE_MAHONY_H
#define AIRMOUSE_MAHONY_H



void Mahony_Init(void);
void Mahony_Updata(float gx,float gy,float gz,
                   float ax,float ay,float az,
                   float dt);
void Mahony_GetEuler(float *roll, float *pitch, float *yaw);

#endif //AIRMOUSE_MAHONY_H