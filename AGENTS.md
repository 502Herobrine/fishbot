# FishBot 单片机程序开发约定

本文档适用于 `/home/nbclass/Documents/PlatformIO/Projects/motor_control` 这个
PlatformIO ESP32 固件项目。它记录项目所有者对代码复用、库组织、注释、实验安全和
上位机联调的要求。除非用户明确授权，单片机程序应保持稳定，不能为了验证上位机而
随意修改。

## 1. 总体开发习惯

- 默认使用中文沟通。先阅读源码和 `lib` 目录，再解释根因、列方案和修改文件；不要凭
  函数名猜测库行为。
- 用户非常重视“干净且符合原先代码风格”的实现。已有 API 能解决的问题必须复用已有
  API，不能复制一份莫名其妙的新逻辑。
- 修改应尽量小而明确。不要顺手重排无关代码、替换外部库、改变引脚、网络、PID 或
  消息接口。
- 用户没有明确要求固件修改时，优先只改上位机；特别是 ROS 2、Gazebo、URDF 和 launch
  实验不能通过在固件中增加重复发布者来“凑通”。
- 保留用户已有未提交修改和实验状态。不要使用 `git reset --hard`、`git checkout --`
  或递归删除来清理工作区；不要在用户未要求时提交代码。
- 任何涉及电机输出、PID 目标、编码器符号、网络凭据或硬件引脚的改动，都必须先说明
  风险和验证方法。

## 2. 项目结构和职责

- 配套 ROS 2 上位机工作空间根目录为
  `/home/nbclass/SRTP/fishbot_ws`。需要确认 URDF、Gazebo、launch、TF、QoS 或导航行为
  时，必须到该目录阅读实际包和配置，不要在单片机工程中猜测上位机实现。
- `platformio.ini` 是当前构建入口，环境为 `fishibot`、ESP32 DevKit、Arduino 框架、
  `micro_ros_platformio` Wi-Fi transport。构建配置变更必须说明对固件、依赖下载和烧录
  的影响。
- `src/main.cpp` 负责硬件初始化、FreeRTOS/micro-ROS 任务编排、ROS 消息收发和调用
  私有库；它不是存放所有运动学、PID 或编码器算法的地方。
- `lib/Kinematics/Kinematics.h/.cpp` 负责轮式运动学、编码器增量、轮速、里程计和累计
  行程；`lib/PidController/PidController.h/.cpp` 负责 PID 状态、目标、增益、积分和
  输出限幅。修改这些功能时先检查对应库，再决定是否需要补 API。
- `lib/README` 记录 PlatformIO 私有库目录约定。每个独立库应有自己的目录，并将接口
  声明放在 `.h`、实现放在 `.cpp`。
- `Esp32McpwmMotor`、`Esp32PcntEncoder` 和 micro-ROS PlatformIO 库是外部依赖。除非
  用户明确要求，不要在项目中复制或改写它们的实现。

## 3. 新函数和 API 的硬性要求

- 如果创建新函数，必须放进正确的库文件：
  - 运动学、编码器计数转换、里程计相关：`lib/Kinematics/Kinematics.h/.cpp`；
  - PID 目标、增益、复位、限幅或控制计算相关：`lib/PidController/PidController.h/.cpp`；
  - 其他明确独立的领域：新建对应的 `lib/<LibraryName>/<LibraryName>.h/.cpp`。
- 不得把本应属于库的函数直接写进 `src/main.cpp`，也不得在 `main.cpp` 里复制一份
  `Kinematics` 或 `PidController` 的计算来绕开现有接口。
- 新 API 必须先检查是否可以组合已有函数；若确实需要新增，接口命名、单位、参数顺序、
  错误处理和注释要与原库一致，并让主程序只调用库接口。
- 库接口变更后检查所有调用点，不要留下旧 API 和新 API 两套行为不同的实现。

## 4. 当前运动控制约定

- 电机/编码器索引固定为：`id == 0` 左轮，`id == 1` 右轮。ROS 关节名称固定为
  `left_wheel_joint`、`right_wheel_joint`；数组顺序、左右含义和符号不能在不同模块中
  各自解释。
- `Kinematics` 当前主要单位为毫米体系：轮子半径为 32 mm，轮距为 175 mm，轮速为
  mm/s，角速度为 rad/s，发布 JointState 时用累计行程除以轮半径得到 rad。新增代码
  必须在接口和注释中明确单位，不要混用 m、mm 或脉冲。
- `Kinematics` 的既有职责包括：
  `set_motor_param`、`set_wheel_distance`、`kinematics_inverse`、
  `kinematics_forward`、`update_motor_speed`、`get_motor_speed`、
  `get_motor_distance`、`update_odom` 和 `get_odom`。先复用这些接口。
- `PidController` 的既有职责包括：
  `update`、`update_target`、`update_pid`、`reset` 和 `out_limit`。不要在主循环中
  另写一套 PID、积分限幅或输出限幅。
- `loop()` 更新编码器和运动学，micro-ROS 任务发布消息并处理 `/cmd_vel`；两者共享
  `Kinematics` 和 PID 状态时必须继续使用 `motion_control_mutex`，不能读写一半的数据。
- 固件启动时目标值和电机输出必须保持安全的零状态。开环、闭环、手动转轮和 Gazebo
  同步是不同实验模式，启用某种模式前必须明确哪些输出代码会重新生效。

## 5. micro-ROS 和话题约定

- ESP32 是 micro-ROS 客户端，主机运行 `micro_ros_agent`。网络 SSID、密码和主机地址
  属于运行配置和敏感信息，不要写入 AGENTS、提交信息或公开日志；变更前确认实际网络
  拓扑。
- 当前固件订阅 `/cmd_vel`，发布 `/odom` 和 `/joint_states`。`/joint_states` 已由固件
  发布真实编码器轮角，上位机 `robot_state_publisher` 和 Gazebo 插件直接使用；不要再
  增加第二个固件发布者或改变话题名称来绕过问题。
- `/cmd_vel` 的订阅 QoS、`/odom` 的发布 QoS 和 `/joint_states` 的可靠发布 QoS 必须
  与上位机端匹配。修改 QoS 前先检查 `ros2 topic info -v` 和对应插件的订阅要求。
- `JointState` 使用 micro-ROS utilities 预先分配动态序列内存；名称、position、velocity
  的容量和 size 必须保持一致，`effort` 当前不使用。不要在高频发布回调中反复分配、
  释放或改变消息数组所有权。
- `/odom` 和 `/joint_states` 共用同一批编码器采样的时间戳；修改时间同步、定时器周期
  或消息填充顺序时，说明会如何影响 TF、Gazebo 和导航。

## 6. 注释和代码风格

- 复杂逻辑必须使用详细中文注释，重点解释单位、左右轮约定、编码器到距离/角度的换算、
  QoS 选择、任务并发、互斥锁和为什么不能使用另一种实现。
- 注释应和代码一致；改了行为就同步更新注释，不要留下“已实现但实际没有执行”的说明。
- 保持现有 C++ 风格、命名和文件组织。不要为了满足某个格式化工具而对整个历史文件
  做无关重排。
- 不要把错误吞掉。初始化、内存分配、发布、任务创建和硬件接口失败时，应记录清晰
  错误并进入安全状态；不要继续用未初始化的电机、编码器或 ROS 消息。
- 高速循环和 micro-ROS 回调中避免阻塞、动态分配和不可控的长时间打印。调试输出应有
  明确目的，实验结束后不要把大量串口打印留在实时路径。

## 7. 硬件实验安全

- 修改、烧录或运行电机控制代码前，先让车轮离地，确认电机目标为零，并准备断电/急停
  手段。不要在没有用户明确要求时执行 `pio run -t upload`、发送 `/cmd_vel` 或启用
  电机输出。
- 任何可能让上电后电机转动、让电机保持上次目标、改变 PID 输出限幅、改变编码器方向
  或修改引脚映射的变化，都必须先进行静态检查并向用户说明。
- 手动转轮实验优先观察编码器计数、`Kinematics::get_motor_distance`、`/joint_states`
  和 Gazebo 反馈；不要因为 Gazebo 方向不符就直接修改固件符号，先确认 URDF 关节轴、
  左右轮索引、编码器方向和上位机插件。
- 开环实验应明确注释/恢复哪些闭环输出代码，不能只注释 PID 的一个调用而留下另一处
  电机写入；恢复前检查两个轮子的目标、限幅和零速行为。

## 8. 推荐验证流程

1. 查看 `git status`，确认没有覆盖用户的实验修改。
2. 阅读 `src/main.cpp` 和相关 `lib` 的完整接口及实现，确认新需求能否用现有 API 完成。
3. 先在轮子离地、输出为零的条件下编译：

   ```bash
   pio run
   ```

4. 如果用户明确要求烧录，再单独执行烧录并说明端口和影响；烧录后使用 PlatformIO
   monitor 或串口日志确认初始化、网络连接、Agent 连接和时间同步。
5. 上位机联调时确认 `/cmd_vel`、`/odom`、`/joint_states` 的名称、QoS、数组顺序、时间戳
   和频率；再观察 Gazebo 的 `/gazebo_joint_states`。
6. 报告中区分“编译通过”“串口初始化通过”“micro-ROS 已连接”和“真实轮子/Gazebo
   已实测”。没有执行过的硬件动作不能描述为已验证。
