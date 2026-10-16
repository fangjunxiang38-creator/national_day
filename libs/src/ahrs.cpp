#include "ahrs.hpp"
#include <cmath>

#define RAD2DEG 57.29577951308232f

void cAHRS::Init()
{
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    integral[0] = integral[1] = integral[2] = 0.0f;
    roll = pitch = yaw = 0.0f;
}

void cAHRS::Update(float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
{
    /* 1. 加速度只取方向；模长接近 0（自由落体/剧烈运动）时跳过修正 */
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.001f) { ax /= norm; ay /= norm; az /= norm; }
    else               { ax = 0.0f;   ay = 0.0f;   az = 0.0f;   }

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    /* 2. 用四元数估计"重力方向在机体坐标下应该指向哪" */
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = 1.0f - 2.0f * (q1*q1 + q2*q2);

    /* 3. 估计值与测量值叉积 = 姿态误差 */
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    /* 4. PI 修正：把误差反馈到角速度上 */
    if (ki > 0.0f)
    {
        integral[0] += ki * ex * dt;
        integral[1] += ki * ey * dt;
        integral[2] += ki * ez * dt;
    }
    gx += kp * ex + integral[0];
    gy += kp * ey + integral[1];
    gz += kp * ez + integral[2];

    /* 5. 四元数积分：q += 0.5 * q ⊗ ω * dt */
    float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy) * dt;
    float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx) * dt;
    float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx) * dt;
    q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

    /* 6. 归一化 */
    float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= n; q1 /= n; q2 /= n; q3 /= n;
    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;

    /* 7. 输出欧拉角（度） */
    roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;

    float sp = 2.0f*(q0*q2 - q3*q1);            /* sin(pitch) 要限幅再 asin */
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;
    pitch = asinf(sp) * RAD2DEG;

    yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}