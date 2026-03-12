#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController {
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    // 可调 PID 参数
    float target_;
    float out_min_;
    float out_max_;
    float kp_;
    float ki_;
    float kd_;
    // pid 中间过程值
    float error_sum_;
    float derror_;
    float error_last_;
    float error_pre_;
    float integral_up_ = 2500; // 积分上限

public:
    float update(float current); // 提供当前值，返回下次输出值，也就是PID的结果
    void update_target(float target); // 更新目标值
    void update_pid(float kp, float ki, float kd); // 更新PID增益
    void reset(); // 重置PID
    void out_limit(float min, float max); // 输出限幅

};

#endif