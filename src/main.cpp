#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include "PidController.h"
#include "Kinematics.h"

Esp32McpwmMotor motor; // 创建一个名为motor的对象，用于控制电机
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PidController pid_controller[2];// 创建两个PID控制器
Kinematics kinematics; // 创建一个Kinematics对象

float target_linear_speed = 50.0f; // 目标线速度，单位mm/s
float target_angular_speed = 0.1f; // 目标角速度，单位rad/s
float out_left_speed;
float out_right_speed;

void setup()
{
  // 1.初始化串口
  Serial.begin(115200); // 初始化串口通信，设置通信速率为115200

  // 2. 设置电机引脚
  motor.attachMotor(0, 22, 23); // 将电机0连接到引脚22和引脚23
  motor.attachMotor(1, 12, 13); // 将电机1连接到引脚12和引脚13

  // 3.设置编码器
  encoders[0].init(0, 32, 33); // 初始化第一个编码器，使用GPIO 32和33连接
  encoders[1].init(1, 26, 25); // 初始化第二个编码器，使用GPIO 26和25连接
  
  // 4. 设置PID控制器参数
  pid_controller[0].update_pid(0.625, 0.125, 0.00); // 设置第一个PID控制器的参数
  pid_controller[1].update_pid(0.625, 0.125, 0.00); // 设置第二个PID控制器的参数
  pid_controller[0].out_limit(-100, 100); 
  pid_controller[1].out_limit(-100, 100);
  pid_controller[0].update_target(100);
  pid_controller[1].update_target(100);

  // 5. 初始化轮子间距和电动机参数
  kinematics.set_wheel_distance(175.0f);
  kinematics.set_motor_param(0, 0.1051566);
  kinematics.set_motor_param(1, 0.1051566);

  // 6. 运动学逆解并设置速度
  kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, out_left_speed, out_right_speed);
  pid_controller[0].update_target(out_left_speed);
  pid_controller[1].update_target(out_right_speed);
}

void loop()
{
  delay(10); // 等待10毫秒
  kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks()); // 更新电动机速度和编码器数据
  motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0))); // 更新电机0的速度
  motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1))); // 更新电机1的速度
  Serial.printf("x=%f, y=%f, angle=%f\n", kinematics.get_odom().x, kinematics.get_odom().y, kinematics.get_odom().angle); // 打印当前位姿信息
}
