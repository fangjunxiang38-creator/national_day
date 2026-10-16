#ifndef AHRS_HPP
#define AHRS_HPP

/**
 * @brief 姿态解算（6 轴：陀螺 + 加速度计）
 * @note  纯算法，不依赖 HAL：输入角速度/加速度，输出欧拉角
 */
class cAHRS
{
public:
    void Init();

    /**
     * @brief 更新一次姿态
     * @param gx,gy,gz 角速度，单位 rad/s（需已扣零偏）
     * @param ax,ay,az 加速度，单位 m/s²（只用方向）
     * @param dt 距上次调用的时间，单位 s
     */
    void Update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt);

    /* 互补滤波增益（可调） */
    float kp = 1.0f;
    float ki = 0.0f;

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          /* 四元数 w,x,y,z */
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;     /* 欧拉角，单位：度 */

private:
    float integral[3] = {0.0f, 0.0f, 0.0f};
};

#endif // AHRS_HPP