#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include "PidController.h"
#include "Kinematics.h"
// 引入 micro-ROS 和 wifi相关头文件
#include <micro_ros_platformio.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>

Esp32McpwmMotor motor; // 创建一个名为motor的对象，用于控制电机
Esp32PcntEncoder encoders[2]; // 创建一个数组用于存储两个编码器
PidController pid_controller[2];// 创建两个PID控制器
Kinematics kinematics; // 创建一个Kinematics对象

float out_left_speed;
float out_right_speed;

// 声明相关的结构体对象
rcl_allocator_t allocator; // 内存分配器
rclc_support_t support; // 用于存储时钟、内存分配器和上下文，提供支持
rclc_executor_t executor; // 执行器，用于管理订阅和计时器回调的执行
rcl_node_t node; // 节点
rcl_subscription_t subscriber; // 订阅者
geometry_msgs__msg__Twist sub_msg; // 存储接收到的速度消息
rcl_publisher_t odom_publisher; // 发布者
nav_msgs__msg__Odometry odom_msg; // 存储要发布的里程计消息
rcl_timer_t timer; // 定时器，可以定时调用某个函数

void twist_callback(const void *msg_in) {
    // 将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型
    const geometry_msgs__msg__Twist *twist_msg =
        (const geometry_msgs__msg__Twist *)msg_in;

    // 运动学逆解并设置速度
    kinematics.kinematics_inverse(twist_msg->linear.x * 1000, twist_msg->angular.z,
                                 out_left_speed, out_right_speed);
    pid_controller[0].update_target(out_left_speed);
    pid_controller[1].update_target(out_right_speed);
}

// 在定时器回调函数中完成话题发布
void callback_publisher(rcl_timer_t *timer, int64_t last_call_time) {
    odom_t odom = kinematics.get_odom();      // 获取里程计数据
    int64_t stamp = rmw_uros_epoch_millis();  // 获取当前系统时间 (毫秒)

    // 设置消息的时间戳
    odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
    // 纳秒部分：取毫秒余数并转换为纳秒
    odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6);

    // 设置位置（Position）
    odom_msg.pose.pose.position.x = odom.x;
    odom_msg.pose.pose.position.y = odom.y;

    // 将偏航角（Yaw）转换为四元数（Orientation）
    odom_msg.pose.pose.orientation.w = cos(odom.angle * 0.5);
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = sin(odom.angle * 0.5);

    // 设置速度（Twist）
    odom_msg.twist.twist.angular.z = odom.angle_speed;
    odom_msg.twist.twist.linear.x = odom.linear_speed;

    // 发布里程计话题
    if (rcl_publish(&odom_publisher, &odom_msg, NULL) != RCL_RET_OK) {
        Serial.printf("error: odom publisher failed!\n");
    }
}

// 单独创建一个任务运行 micro-ROS , 相当于一个线程
void micro_ros_task(void *parameter) {
  // 1. 设置传输协议并延时等待设置完成
  IPAddress agent_ip;
  agent_ip.fromString("10.181.161.70"); // 替换为你自己主机的 IP 地址
  set_microros_wifi_transports("NBclass", "3116Herobrine", agent_ip, 8888);
  delay(2000);
  // 2. 初始化内存分配器
  allocator = rcl_get_default_allocator();
  // 3. 初始化 support
  rclc_support_init(&support, 0, NULL, &allocator);
  // 4. 初始化节点 fishbot_motion_control
  rclc_node_init_default(&node, "fishbot_motion_control", "", &support);
  // 5. 初始化执行器；初始化订阅者并添加到执行器中
  unsigned int num_handles = 0+2;
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_subscription_init_best_effort(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);
  // 6. 初始化发布者和定时器
  odom_msg.header.frame_id = 
      micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = 
      micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
  rclc_publisher_init_best_effort(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
  // 7. 时间同步
  while (!rmw_uros_epoch_synchronized()) {  // 如果没有同步
      rmw_uros_sync_session(1000);  // 尝试进行时间同步
      delay(10);
  }
  // 8. 创建定时器，间隔 50 ms 发布调用一次 callback_publisher 发布里程计话题
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), callback_publisher);
  // 将定时器添加到执行器中
  rclc_executor_add_timer(&executor, &timer);

  // 循环执行器
  rclc_executor_spin(&executor);
}


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

  // 创建任务运行 micro_ros_task
  xTaskCreate(micro_ros_task,    // 任务函数
              "micro_ros",      // 任务名称
              10240,            // 任务堆栈大小 (字节)
              NULL,             // 传递给任务函数的参数
              1,                // 任务优先级
              NULL              // 任务句柄
  );
}

void loop()
{
  delay(10); // 等待10毫秒
  kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks()); // 更新电动机速度和编码器数据
  motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0))); // 更新电机0的速度
  motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1))); // 更新电机1的速度
  Serial.printf("x=%f, y=%f, angle=%f\n", kinematics.get_odom().x, kinematics.get_odom().y, kinematics.get_odom().angle); // 打印当前位姿信息
}
