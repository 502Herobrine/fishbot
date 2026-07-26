#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
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
#include <sensor_msgs/msg/joint_state.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

// 轮子半径，单位mm。该值与fishbot.urdf中的0.032m保持一致，并与Kinematics使用相同的长度单位
constexpr float WHEEL_RADIUS_MM = 32.0f;
// 左右轮数量。JointState中的关节名称、位置和速度数组都必须保持相同长度
constexpr size_t WHEEL_COUNT = 2;

Esp32McpwmMotor motor; // 创建一个名为motor的对象，用于控制电机
Esp32PcntEncoder encoders[WHEEL_COUNT]; // 创建一个数组用于存储两个编码器
PidController pid_controller[WHEEL_COUNT];// 创建两个PID控制器
Kinematics kinematics; // 创建一个Kinematics对象

// Arduino的loop任务和micro-ROS任务都会访问运动学与PID对象，使用互斥锁防止两个任务同时读写这些对象
SemaphoreHandle_t motion_control_mutex = NULL;

// 声明相关的结构体对象
rcl_allocator_t allocator; // 内存分配器
rclc_support_t support; // 用于存储时钟、内存分配器和上下文，提供支持
rclc_executor_t executor; // 执行器，用于管理订阅和计时器回调的执行
rcl_node_t node; // 节点
rcl_subscription_t subscriber; // 订阅者
geometry_msgs__msg__Twist sub_msg; // 存储接收到的速度消息
rcl_publisher_t odom_publisher; // 发布者
nav_msgs__msg__Odometry odom_msg; // 存储要发布的里程计消息
rcl_publisher_t joint_state_publisher; // 左右轮关节状态发布者
sensor_msgs__msg__JointState joint_state_msg; // 存储左右轮转角和转速消息
rcl_timer_t timer; // 定时器，可以定时调用某个函数

void twist_callback(const void *msg_in) {
    // 将接收到的消息指针转化为 geometry_msgs__msg__Twist 类型
    const geometry_msgs__msg__Twist *twist_msg =
        (const geometry_msgs__msg__Twist *)msg_in;
    float out_left_speed;
    float out_right_speed;

    // 运动学逆解并设置速度
    kinematics.kinematics_inverse(twist_msg->linear.x * 1000, twist_msg->angular.z,
                                 out_left_speed, out_right_speed);

    // loop任务可能正在计算PID输出，修改PID目标值前先取得同一个运动控制互斥锁
    xSemaphoreTake(motion_control_mutex, portMAX_DELAY);
    pid_controller[0].update_target(out_left_speed);
    pid_controller[1].update_target(out_right_speed);
    xSemaphoreGive(motion_control_mutex);
}

// 在定时器回调函数中完成话题发布
void callback_publisher(rcl_timer_t *timer, int64_t last_call_time) {
    int64_t stamp = rmw_uros_epoch_millis();  // 获取当前系统时间 (毫秒)
    odom_t odom;
    float left_distance;
    float right_distance;
    int16_t left_speed;
    int16_t right_speed;

    // loop任务负责更新Kinematics，取得互斥锁后直接通过库接口读取同一次更新得到的里程计和轮子状态
    xSemaphoreTake(motion_control_mutex, portMAX_DELAY);
    odom = kinematics.get_odom();
    left_distance = kinematics.get_motor_distance(0);
    right_distance = kinematics.get_motor_distance(1);
    left_speed = kinematics.get_motor_speed(0);
    right_speed = kinematics.get_motor_speed(1);
    xSemaphoreGive(motion_control_mutex);

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

    // joint_states直接复用odom的时间戳，保证同一批编码器数据在ROS 2中具有一致的采样时间
    joint_state_msg.header.stamp = odom_msg.header.stamp;

    // Kinematics返回累计行程，轮子转角 = 累计行程 / 轮子半径，mm相除后得到rad
    // 直接使用编码器累计行程，不在发布回调中再次积分速度，避免发布周期抖动造成角度累积误差
    joint_state_msg.position.data[0] = left_distance / WHEEL_RADIUS_MM;
    joint_state_msg.position.data[1] = right_distance / WHEEL_RADIUS_MM;

    // Kinematics返回轮子线速度，轮子角速度 = 线速度 / 轮子半径，计算结果单位为rad/s
    joint_state_msg.velocity.data[0] = left_speed / WHEEL_RADIUS_MM;
    joint_state_msg.velocity.data[1] = right_speed / WHEEL_RADIUS_MM;

    // 发布真实编码器对应的左右轮关节状态，robot_state_publisher和Gazebo将同时订阅该话题
    if (rcl_publish(&joint_state_publisher, &joint_state_msg, NULL) !=
        RCL_RET_OK) {
        Serial.printf("error: joint state publisher failed!\n");
    }
}

// 单独创建一个任务运行 micro-ROS , 相当于一个线程
void micro_ros_task(void *parameter) {
  // 1. 设置传输协议并延时等待设置完成
  IPAddress agent_ip;
  agent_ip.fromString("192.168.0.103"); // 替换为你自己主机的 IP 地址
  set_microros_wifi_transports("TP-LINK_1E54", "3116Herobrine", agent_ip, 8888);
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
  // 6. 初始化里程计消息和里程计发布者
  odom_msg.header.frame_id = 
      micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = 
      micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
  rclc_publisher_init_best_effort(
      &odom_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");

  // 7. 初始化左右轮关节状态消息，并创建可靠的joint_states发布者
  // JointState中包含多个动态数组，使用micro-ROS已有的消息内存工具统一完成初始化
  micro_ros_utilities_memory_conf_t memory_conf =
      micro_ros_utilities_memory_conf_default;
  // name属于ROS 2字符串序列，position、velocity和effort属于基础类型序列，容量都只需要容纳左右两个轮子
  memory_conf.max_ros2_type_sequence_capacity = WHEEL_COUNT;
  memory_conf.max_basic_type_sequence_capacity = WHEEL_COUNT;
  if (!micro_ros_utilities_create_message_memory(
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          &joint_state_msg, memory_conf)) {
    Serial.printf("error: joint state message init failed!\n");
    vTaskDelete(NULL);
    return;
  }

  // 内存工具只设置数组容量，发布前还需要明确指定本次消息中实际使用的元素数量
  joint_state_msg.name.size = WHEEL_COUNT;
  joint_state_msg.position.size = WHEEL_COUNT;
  joint_state_msg.velocity.size = WHEEL_COUNT;
  joint_state_msg.effort.size = 0;

  // 关节名称必须与fishbot.urdf中的名称完全相同，否则robot_state_publisher和Gazebo无法找到对应关节
  joint_state_msg.name.data[0] = micro_ros_string_utilities_set(
      joint_state_msg.name.data[0], "left_wheel_joint");
  joint_state_msg.name.data[1] = micro_ros_string_utilities_set(
      joint_state_msg.name.data[1], "right_wheel_joint");

  // ESP32刚启动时将左右轮的转角和转速都初始化为0
  joint_state_msg.position.data[0] = 0.0;
  joint_state_msg.position.data[1] = 0.0;
  joint_state_msg.velocity.data[0] = 0.0;
  joint_state_msg.velocity.data[1] = 0.0;

  // Gazebo关节位置插件使用可靠QoS订阅，因此这里不能使用best_effort发布者，否则DDS会判定QoS不兼容
  if (rclc_publisher_init_default(
          &joint_state_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "/joint_states") != RCL_RET_OK) {
      Serial.printf("error: joint state publisher init failed!\n");
      vTaskDelete(NULL);
      return;
  }

  // 8. 时间同步
  while (!rmw_uros_epoch_synchronized()) {  // 如果没有同步
      rmw_uros_sync_session(1000);  // 尝试进行时间同步
      delay(10);
  }
  // 9. 创建定时器，间隔50ms同时发布里程计和左右轮关节状态
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
  pid_controller[0].update_target(0);
  pid_controller[1].update_target(0);

  // 5. 初始化轮子间距和电动机参数
  kinematics.set_wheel_distance(175.0f);
  kinematics.set_motor_param(0, 0.1051566);
  kinematics.set_motor_param(1, 0.1051566);

  // 创建运动控制互斥锁，保护loop任务与micro-ROS任务共同访问的Kinematics和PID对象
  motion_control_mutex = xSemaphoreCreateMutex();
  if (motion_control_mutex == NULL) {
    Serial.printf("error: motion control mutex init failed!\n");
    while (true) {
      delay(1000);
    }
  }

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
//   float left_motor_output;
//   float right_motor_output;
  odom_t odom;

  // 在同一个临界区内更新运动学状态、读取轮速并计算PID输出，避免micro-ROS任务读到更新一半的数据
  xSemaphoreTake(motion_control_mutex, portMAX_DELAY);
  kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks()); // 更新电动机速度和编码器数据
//   left_motor_output = pid_controller[0].update(kinematics.get_motor_speed(0));
//   right_motor_output = pid_controller[1].update(kinematics.get_motor_speed(1));
  odom = kinematics.get_odom();
  xSemaphoreGive(motion_control_mutex);

//   motor.updateMotorSpeed(0, left_motor_output); // 更新电机0的速度
//   motor.updateMotorSpeed(1, right_motor_output); // 更新电机1的速度
  Serial.printf("x=%f, y=%f, angle=%f\n", odom.x, odom.y, odom.angle); // 打印当前位姿信息
}
