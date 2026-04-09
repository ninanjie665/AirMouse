//
// Created by Administrator on 2026/4/9.
//
#include "mouse.h"
#include "math.h"
#include "usbd_hid.h"

#define SENSITIVITY     13.6f     // 灵敏度（像素/度/秒），需实测调整
#define DT              0.01f    // 定时器周期 10ms = 0.01秒
#define DEAD_ZONE       1.5f     // 死区阈值（dps），小于此值忽略
#define DEADZONE_DEG       2.0f      // 死区（度）
#define ANGLE_MAX          30.0f     // 最大有效角度（度）
#define SPEED_MAX_PX_PER_S 400.0f    // 最大移动速度（像素/秒）

extern USBD_HandleTypeDef hUsbDeviceFS;

void Gyro_To_MouseMove() {
    float wx, wy, wz;
    float dx_f = 0, dy_f = 0;

    MPU6050_GetGyroDPS(&wx,&wy,&wz);

    if (fabsf(wx) <= DEAD_ZONE) wx = 0.0f;
    if (fabsf(wy) <= DEAD_ZONE) wy = 0.0f;
    if (fabsf(wz) <= DEAD_ZONE) wz = 0.0f;

    dx_f = wz * SENSITIVITY * DT ;
    dy_f = wx * SENSITIVITY * DT ;

    if (dx_f > 100.0f) dx_f = 100.0f;
    if (dx_f < -100.0f) dx_f = -100.0f;
    if (dy_f > 100.0f) dy_f = 100.0f;
    if (dy_f < -100.0f) dy_f = -100.0f;

    int16_t dx  = 0;
    dx = (int16_t)dx_f;
    int16_t dy = 0;
    dy = (int16_t)dy_f;

    uint8_t report[4] = {0};

    // report[0] : 按键 (bit0:左键, bit1:右键, bit2:中键)
    // 如果你还没有按键，就保持 0
    report[0] = 0;

    // report[1] : X 轴位移 (范围 -127 ~ 127)
    // 注意强制转为 uint8_t，超出范围会截断，最好先限幅
    if (dx > 127) dx = 127;
    if (dx < -127) dx = -127;
    report[1] = (uint8_t)(-dx);

    // report[2] : Y 轴位移
    if (dy > 127) dy = 127;
    if (dy < -127) dy = -127;
    report[2] = (uint8_t)(-dy);

    // report[3] : 滚轮（暂时不用，填 0）
    report[3] = 0;

    USBD_HID_SendReport(&hUsbDeviceFS,report,4);

}
void Angal_To_MouseMove() {
    float gx_dps, gy_dps, gz_dps;
    float ax, ay, az;

    MPU6050_GetGyroDPS(&gx_dps, &gy_dps, &gz_dps);
    MPU6050_GetAccel(&ax, &ay, &az);

    float gx = gx_dps * M_PI / 180.0f;
    float gy = gy_dps * M_PI / 180.0f;
    float gz = gz_dps * M_PI / 180.0f;

    Mahony_Updata(gx, gy, gz, ax, ay, az, DT);

    float roll_rad, pitch_rad, yaw_rad;
    Mahony_GetEuler(&roll_rad, &pitch_rad, &yaw_rad);
    // 关键：转度数
    float roll_deg = roll_rad * 57.29578f;
    float pitch_deg = pitch_rad * 57.29578f;

    // 直接用角度差（零点为0°）
    float roll_diff = roll_deg;
    float pitch_diff = pitch_deg;

    // 死区判断用度数
    float abs_roll = fabsf(roll_diff);
    float vx = 0.0f;
    if (abs_roll > DEADZONE_DEG) {
        float t = (abs_roll - DEADZONE_DEG) / (ANGLE_MAX - DEADZONE_DEG);
        if (t > 1.0f) t = 1.0f;
        vx = t * SPEED_MAX_PX_PER_S;
        if (roll_diff < 0) vx = -vx;
    }

    float abs_pitch = fabsf(pitch_diff);
    float vy = 0.0f;
    if (abs_pitch > DEADZONE_DEG) {
        float t = (abs_pitch - DEADZONE_DEG) / (ANGLE_MAX - DEADZONE_DEG);
        if (t > 1.0f) t = 1.0f;
        vy = t * SPEED_MAX_PX_PER_S;
        if (pitch_diff < 0) vy = -vy;
    }

    float dx_f = vx * DT;
    float dy_f = vy * DT;

    // 累积小数值，避免截断损失
    static float accum_dx = 0, accum_dy = 0;
    accum_dx += dx_f;
    accum_dy += dy_f;
    int16_t dx = (int16_t)accum_dx;
    int16_t dy = (int16_t)accum_dy;
    accum_dx -= dx;
    accum_dy -= dy;

    if (dx > 127) dx = 127; if (dx < -127) dx = -127;
    if (dy > 127) dy = 127; if (dy < -127) dy = -127;

    uint8_t report[4] = {0};
    report[0] = 0;
    report[1] = (uint8_t)(-dx);
    report[2] = (uint8_t)(-dy);
    report[3] = 0;
    USBD_HID_SendReport(&hUsbDeviceFS, report, 4);
}