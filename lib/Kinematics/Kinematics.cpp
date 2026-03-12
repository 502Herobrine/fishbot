#include "Kinematics.h"

void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance) {
    motor_param_[id].per_pulse_distance = per_pulse_distance;
}
void Kinematics::set_wheel_distance(float wheel_distance) {
    wheel_distance_ = wheel_distance;
}
// 传入电机的编号id,返回当前电机的速度
int16_t Kinematics::get_motor_speed(uint8_t id) {
    return motor_param_[id].motor_speed;
}

/**
     * @brief 更新电动机速度和编码器数据
     * @param current_time 当前时间，单位ms
     * @param left_tick 左轮编码器计数值
     * @param right_tick 右轮编码器计数值
     */
void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick) {
    // 计算出上次更新以来经过的时间 dt
    uint32_t dt = current_time - last_update_time_;
    last_update_time_ = current_time;

    //计算电动机1和电动机2的编码器读数变化量 dtick1 和 dtick2
    int32_t dtick1 = left_tick - motor_param_[0].last_encoder_tick;
    int32_t dtick2 = right_tick - motor_param_[1].last_encoder_tick;
    motor_param_[0].last_encoder_tick = left_tick;
    motor_param_[1].last_encoder_tick = right_tick;

    //轮子速度计算
    motor_param_[0].motor_speed = float(dtick1 * motor_param_[0].per_pulse_distance) / dt * 1000.0f;
    motor_param_[1].motor_speed = float(dtick2 * motor_param_[1].per_pulse_distance) / dt * 1000.0f;

    // 更新里程计数据
    update_odom(dt);
}

/**
     * @brief 正运动学计算，输入线速度和角速度，输出左右轮的速度
     * @param left_speed 左轮速度，单位mm/s
     * @param right_speed 右轮速度，单位mm/s
     * @param[out] out_linear_speed 线速度，单位mm/s
     * @param[out] out_angle_speed 角速度，单位rad/s
     */
void Kinematics::kinematics_forward(float left_speed, float right_speed, float &out_linear_speed, float &out_angle_speed) {
    out_linear_speed = (left_speed + right_speed) / 2.0f;
    out_angle_speed = (right_speed - left_speed) / wheel_distance_;
}

/**
     * @brief 逆运动学计算，输入线速度和角速度，输出左右轮的速度
     * @param linear_speed 线速度，单位mm/s
     * @param angle_speed 角速度，单位rad/s
     * @param[out] out_left_speed 左轮速度，单位mm/s
     * @param[out] out_right_speed 右轮速度，单位mm/s
     */
void Kinematics::kinematics_inverse(float linear_speed, float angle_speed, float &out_left_speed, float &out_right_speed) {
    out_left_speed = linear_speed - angle_speed * wheel_distance_ / 2.0f;
    out_right_speed = linear_speed + angle_speed * wheel_distance_ / 2.0f;
}


odom_t &Kinematics::get_odom() {
    return odom_;
}

//用于将角度转换到 -pi 到 pi 之间
void Kinematics::TransAngleInPi(float angle, float &out_angle) {
    // 如果 angle 大于 pi，则减去 2*pi
    if (angle > PI) {
        out_angle = angle - 2 * PI;
    }
    // 如果 angle 小于 -pi，则加上 2*pi
    else if (angle < -PI) {
        out_angle = angle + 2 * PI;
    }
}

/**
     * @brief 更新里程计数据，根据当前电动机速度和上次更新以来经过的时间 dt 计算机器人位姿和速度
     * @param dt 上次更新以来经过的时间，单位ms
     */
void Kinematics::update_odom(uint16_t dt) {
    float dt_s =(float)dt / 1000.0f; // 将 dt 转换为秒
    // 运动学正解，计算线速度和角速度
    kinematics_forward(motor_param_[0].motor_speed, motor_param_[1].motor_speed, odom_.linear_speed, odom_.angle_speed);
    // 转换线速度单位
    odom_.linear_speed /= 1000.0f;

    // 计算当前角度
    odom_.angle += odom_.angle_speed * dt_s;
    // 将角度值 odom_.yaw 转换到 -pi 到 pi 之间
    Kinematics::TransAngleInPi(odom_.angle, odom_.angle);

    // 计算机器人移动距离和在两轴上的分量并进行累积
    float delta_distance = odom_.linear_speed * dt_s;
    odom_.x += delta_distance * std::cos(odom_.angle);
    odom_.y += delta_distance * std::sin(odom_.angle);
}