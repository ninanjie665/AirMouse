#include "mahony.h"

#include <math.h>

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float Kp = 0.1f,Ki = 0.2f;

void Mahony_Init(void) {
    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
}

void Mahony_Updata(float gx,float gy,float gz,
                    float ax,float ay,float az,float dt) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if ( norm < 1e-6f) return;

    ax /= norm ,ay /= norm,az /= norm;

    // 计算理论重力方向
    vx = 2.0f*(q1*q3 - q0*q2);
    vy = 2.0f*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    if (Ki > 0.0f) {
        exInt = Ki * ex * dt;
        eyInt = Ki * ey * dt;
        ezInt = Ki * ez * dt;
    }
    // 比例修正
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;

    // 四元数更新（一阶龙格-库塔）
    float q0_t = q0, q1_t = q1, q2_t = q2, q3_t = q3;
    q0 = q0_t + (-q1_t*gx - q2_t*gy - q3_t*gz) * (0.5f*dt);
    q1 = q1_t + ( q0_t*gx - q3_t*gy + q2_t*gz) * (0.5f*dt);
    q2 = q2_t + ( q3_t*gx + q0_t*gy - q1_t*gz) * (0.5f*dt);
    q3 = q3_t + (-q2_t*gx + q1_t*gy + q0_t*gz) * (0.5f*dt);

    // 归一化
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm < 1e-6f) return;
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

void Mahony_GetEuler(float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    *pitch = asinf(2.0f*(q0*q2 - q3*q1));
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}