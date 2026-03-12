#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <Arduino.h>

// 定义一个结构体用于存储电动机参数
typedef struct
{
    float per_pulse_distance; // 每个脉冲对应的距离
    int16_t motor_speed; // 电机速度 mm/s
    int64_t last_encoder_tick; // 上一次编码器计数值

} motor_param_t;

// 定义一个类处理机器人运动学
class Kinematics
{
public:
    Kinematics() = default;
    ~Kinematics() = default;

    // 设置电动机参数，包括编号和每个脉冲对应的轮子前进距离
    void set_motor_param(uint8_t id, float per_pulse_distance);
    // 设置轮子间距
    void set_wheel_distance(float wheel_distance);
    // 逆运动学计算，输入线速度和角速度，输出左右轮的速度
    void kinematics_inverse(float linear_speed, float angle_speed, float &out_left_speed, float &out_right_speed);
    // 正运动学计算，输入线速度和角速度，输出左右轮的速度
    void kinematics_forward(float left_speed, float right_speed, float &out_linear_speed, float &out_angle_speed);
    // 更新电动机速度和编码器数据
    void update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick);
    // 获取当前电动机速度
    int16_t get_motor_speed(uint8_t id);

private:
    motor_param_t motor_param_[2]; // 存储两个电动机的参数
    uint64_t last_update_time_; // 上一次更新的时间，单位ms
    float wheel_distance_; // 轮子间距
};

#endif  // __KINEMATICS_H__