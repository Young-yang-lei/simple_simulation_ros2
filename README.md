# simple_simulation_ros2

将 FAPP 的 ROS1 `src/simulation/uav_simulator` 和 `src/simulation/Utils` 迁移到 **ROS2 Humble / Ubuntu 22.04**。本仓库包含原目录下的 16 个包，并保留原 ROS 包名；`fake_drone` 的包名是 `poscmd_2_odom`，`local_sensing` 的包名是 `local_sensing_node`。

提供理想跟踪 `fake` 和四旋翼动力学 `so3` 两种后端，共用位置指令、里程计、局部点云和可视化接口。规划器可通过话题接入。本仓库不包含 FAPP 规划器、mot_mapping、PX4、Gazebo 或真实定位算法。

## 构建

建议在干净终端中只加载 ROS2 Humble，避免已经 source 的其他 workspace 中同名 `quadrotor_msgs` 混入。

```bash
source /opt/ros/humble/setup.bash
cd ~/simulation_ros2
rosdep install --from-paths src --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

依赖包括 rclcpp/rclpy、标准消息、tf2、Eigen、PCL、Armadillo、Boost、RViz2/Qt、pluginlib。`joy` 只用于手柄地面站。默认路径无需 CUDA、SVO、VIKIT 或 ROS1 bridge。

## 启动与发送指令

理想跟踪后端：

```bash
ros2 launch so3_quadrotor_simulator simulator_example.launch.py backend:=fake
```

SO3 动力学后端：

```bash
ros2 launch so3_quadrotor_simulator simulator_example.launch.py backend:=so3
```

可选参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `backend` | `fake` | `fake`：直接跟踪参考；`so3`：经过控制器和动力学 |
| `namespace` | 空 | 为整套仿真增加命名空间，例如 `uav_0` |
| `rviz` | `true` | 是否启动 RViz2；服务器/测试可设 `false` |
| `depth` | `false` | 是否增加 CPU 深度投影和相机位姿输出 |

例如：

```bash
ros2 launch so3_quadrotor_simulator simulator_example.launch.py backend:=so3 namespace:=uav_0 rviz:=false depth:=true
```

另一个已加载 workspace 环境的终端中发送位置参考：

```bash
ros2 topic pub --rate 20 /position_cmd quadrotor_msgs/msg/PositionCommand \
  '{position: {x: 0.5, y: 0.25, z: 1.2}, yaw: 0.0, trajectory_flag: 1}'
```

设置了 `namespace:=uav_0` 时，话题改为 `/uav_0/position_cmd`。

RViz 的三维目标工具发布 `goal` / `goal_with_id`，不会直接变成位置控制指令。需要规划器或目标到指令的转换节点；本仓库不会把鼠标目标自动当作可飞轨迹。

## 运行接口

以下是统一启动入口在自身命名空间下的话题名：

| 话题 | 类型 | 用途 |
|---|---|---|
| `position_cmd` | `quadrotor_msgs/msg/PositionCommand` | 位置、速度、加速度、jerk、yaw 参考 |
| `odometry` | `nav_msgs/msg/Odometry` | 所选后端的状态反馈；每套仿真只有一个发布者 |
| `so3_cmd` | `quadrotor_msgs/msg/SO3Command` | 仅 SO3 后端：期望合力、姿态与增益 |
| `imu` | `sensor_msgs/msg/Imu` | 仅 SO3 后端：姿态、机体系角速度、机体系比力 |
| `map_generator/global_cloud` | `sensor_msgs/msg/PointCloud2` | FAPP 地图生成器的全局场景点云 |
| `map_generator/static_obs_cloud` | 同上 | 局部感知使用的静态障碍地图，可靠 + transient_local |
| `map_generator/obj_cloud` | 同上 | 动态物体点云，接入局部感知的 `local_map` |
| `cloud` | 同上 | 当前探测范围内的局部观测，SensorDataQoS |
| `depth` | `sensor_msgs/msg/Image` | 可选，`32FC1` 米制深度；无观测像素为正无穷 |
| `camera_info` | `sensor_msgs/msg/CameraInfo` | 可选，相机内参 |
| `camera_pose` | `geometry_msgs/msg/PoseStamped` | 可选，世界系中的相机光学坐标系位姿 |
| `robot` / `path` | Marker / Path | 无人机模型和有界轨迹历史 |
| `tf_path` | `nav_msgs/msg/Path` | TF 辅助工具的有界轨迹输出 |

世界 frame 为 `world`，机体 frame 默认为 `[namespace/]base_link`。机体系按 FLU，光学坐标系为 x 右、y 下、z 前；相机默认朝机头，可通过 `camera.translation` 设置机体系安装平移。地图和 odom 必须处于同一个世界 frame，当前感知节点不会自动变换地图 frame。

本后端使用实时墙钟调度，不发布 `/clock`。默认保持 `use_sim_time=false`；暂停、时间倍率、重置和确定性锁步尚未提供。原数学模型把线速度作为世界系量提供给控制器，本次保留此约定；接入要求严格按 `child_frame_id` 表达 twist 的外部组件时，需要转换线速度坐标系。

## 包与迁移后的职责

| 目录 | ROS 包 | 迁移后的内容 |
|---|---|---|
| `uav_simulator/fake_drone` | `poscmd_2_odom` | rclcpp 理想位置跟踪、姿态构造、定时 odom |
| `uav_simulator/local_sensing` | `local_sensing_node` | CPU 局部点云、地图刷新、动态点云合并、可选深度投影 |
| `uav_simulator/map_generator` | `map_generator` | 保留 FAPP 静态/动态障碍与走廊生成算法 |
| `uav_simulator/mockamap` | `mockamap` | 保留 Perlin、柱体、2D/3D 迷宫生成和独立 launch |
| `uav_simulator/so3_control` | `so3_control` | 普通 rclcpp 控制节点及示例指令程序；保留控制方程 |
| `uav_simulator/so3_quadrotor_simulator` | 同名 | 四旋翼刚体/电机动力学、内环和统一启动入口 |
| `uav_simulator/uav_utils` | `uav_utils` | ROS2 消息转换头文件、几何工具和四个 Python3 辅助程序 |
| `uav_simulator/cmake_utils` | `cmake_utils` | ament 兼容包，安装旧 CMake 辅助文件；迁移节点使用现代依赖目标 |
| `Utils/quadrotor_msgs` | `quadrotor_msgs` | 15 种 ROS2 消息和串行编码/解码库 |
| `Utils/pose_utils` | `pose_utils` | 导出 Armadillo 数学库 |
| `Utils/odom_visualization` | `odom_visualization` | 模型、轨迹、速度、协方差等 Marker/消息，TF2 接口 |
| `Utils/rviz_plugins` | `rviz_plugins` | RViz2 三维目标工具；移除未实现的插件声明 |
| `Utils/selected_points_publisher` | 同名 | RViz2 框选无人机、发布选择与目标 |
| `Utils/assign_goals` | `assign_goals` | 按选择的机群相对布局分发目标，增加空选择保护 |
| `Utils/random_goals` | `random_goals` | 从参数候选点随机分配目标，修正随机下标边界 |
| `Utils/manual_take_over` | `manual_take_over` | 手柄转发、停止规划请求和人工位置参考，增加手柄数组检查 |

## 主要改动记录

迁移日期：2026-11-18。

### ROS API、构建与启动

- 16 个包改为 `ament_cmake`、package format 3，添加安装与导出规则，用 colcon 构建。
- C++ 的 NodeHandle、发布/订阅、参数、时间和日志改用 rclcpp；ROS1 消息头和命名空间改为 ROS2 `msg/*.hpp` / `::msg::`。
- Python2/rospy 脚本改为 Python3/rclpy。`topic_statistics.py` 原来是空模板，现在可统计指定话题接收频率和序列化流量。
- XML launch 改为 ROS2 Python launch；参数层级用点号，例如 `simulator.init_state_z`、`gains.kx.x`。控制器 YAML 已改为 `ros__parameters` 结构，并统一原来不匹配的 pos/vel 与 kx/kv 名称。
- ROS1 nodelet 改为普通 `so3_control_node`，无需 nodelet 管理器；本版本未使用 ROS2 组件容器。
- RViz1 插件移植到 `rviz_common` / `rviz_rendering` / `rviz_default_plugins` 与 pluginlib；配置换成 RViz2 格式。
- 采用系统 Boost.Odeint，去除复制的旧 Boost/odeint、生成的 ROS1 Python 消息、备份及编辑器文件。原模型与插件源码中的版权声明保留。

### 闭环与状态修正

- 统一启动入口只选择一种后端，避免原 FAPP quick_start 同时启动 SO3 和 fake，却把规划指令只接到 fake 的混合状态路径。
- 恢复复制源码里被注释的动力学 odom/IMU 发布，补充 IMU 时间戳，去除 frame 名的前导 `/`。
- 控制器与动力学默认使用一致的质量 `0.9 kg`；初始化控制量、外力矩和四元数，处理电机使能命令。
- IMU 加速度改为机体系比力；控制器把它转回世界系运动加速度使用。
- fake_drone 不再把线加速度塞进 `twist.angular`；用参考 jerk/yaw rate 生成角速度近似，并加入零合力/退化姿态保护。
- 动态障碍物位移使用实际时间差，不再固定按 `0.02 s` 更新，避免改变发布频率后运动速度随之变化。
- 修正全局 ROS 句柄的退出释放顺序，并让循环节点复用 executor，避免 Ctrl+C 后创建临时 executor 或静态析构访问失效 DDS 资源；轨迹显示历史限制为 3000 个点。

### 局部感知

- 原 FAPP 默认关闭 CUDA。本版本以 CPU PCL 路径为基础重写为独立 ROS2 节点，支持刷新全局地图和替换动态点云。
- 全局地图订阅采用可靠/transient_local，传感器输出采用 SensorDataQoS；外部地图发布者需提供兼容 QoS。
- 新增可选 CPU 深度投影：同像素取最近点，输出 `depth`、`camera_info`、`camera_pose`。此实现替代旧的可选 CUDA 路径，移除原 CUDA/EuRoC 杂项与 SVO/VIKIT 历史声明。
- 点云模式保留附近点/高度范围筛选，不做完整遮挡射线检测；深度模式是离散点投影，可能有孔洞。输出相机位姿但未广播相机 TF。没有真实 RGB、纹理、噪声、畸变和传感器延迟模型，也不宣称与原 CUDA 输出逐像素一致。

### 自定义消息兼容变化

| ROS1 字段 | ROS2 字段 | 原因 |
|---|---|---|
| `SO3Command.kR` | `k_r` | ROS2 字段命名规则 |
| `SO3Command.kOm` | `k_om` | 同上 |
| `Gains.Kp/Kd/Kp_yaw/Kd_yaw` | `kp/kd/kp_yaw/kd_yaw` | 同上 |
| 隐式 `Header` | `std_msgs/Header` | 显式接口依赖 |
| `Header.seq` | 不存在 | ROS2 标准 Header 移除序号；旧串行 SO3 编码中的 seq 固定为 0 |

编码/解码接口通过 `quadrotor_codecs` 库导出。旧的物理串口协议仅保留编解码，不代表经过真实飞控或硬件验证。ROS1 项目不能直接连接这些 ROS2 消息；外部规划器需要原生迁移或单独配置桥接和字段转换。

## 其他入口

```bash
ros2 launch mockamap mockamap.launch.py
ros2 launch mockamap maze2d.launch.py
ros2 launch mockamap maze3d.launch.py
ros2 launch mockamap perlin3d.launch.py
ros2 launch mockamap post2d.launch.py
ros2 launch random_goals random_goals.launch.py
ros2 launch manual_take_over take_over_drone.launch.py
ros2 launch manual_take_over take_over_station.launch.py
ros2 run uav_utils send_odom.py
ros2 run uav_utils odom_to_euler.py
ros2 run uav_utils topic_statistics.py --ros-args -p topic:=odometry
```

`tf_assist.py` 现在每个实例处理一个 odom 输入，通过命名空间/remap 启动多个实例。参数 `world_frame` 和 `body_frame` 是 frame 别名，不会执行任意空间变换；只在坐标系确实重合时使用。`broadcast_tf` 默认 true，`path_length` 默认 3000。

多机随机目标节点默认读取 `drone_<id>/odometry`；可用对应 namespace 启动后端。人工接管仍沿用原项目的“通知规划器停发，再发布手动指令”约定，需要外部规划器配合，不能把它当作飞控硬件急停。地面站的 `joystick_from_users` 和无人机端 `joystick_from_bridge` 可通过 remap 直连或接入自有通信桥。

## 验证

已在本机 Ubuntu 22.04 / ROS2 Humble 完成 16 个包的 Release 构建。运行测试通过 fake/SO3 指令闭环、SO3 到达后继续保持 5 秒（误差小于 0.15 m）、传感器数据链、地图更新和正常退出；RViz2 两个插件通过加载/构造测试（1 项测试，0 失败）。测试位置指令以 20 Hz 发布。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select rviz_plugins --event-handlers console_direct+
colcon test-result --verbose
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 python3 tests/runtime_smoke.py
```

`tests/runtime_smoke.py` 在独立命名空间启动并关闭子进程，验证：

- fake 与 SO3 位置指令跟踪、单一 odom 发布者、有效姿态和 SO3 IMU。
- 地图 → 局部点云/深度 → 可视化数据链。
- 深度同像素最近点、超范围点剔除、全局地图刷新、动态点云更新。
- mockamap 发布非空且带时间戳的点云。
- 辅助里程计、欧拉角转换、话题频率/流量统计、随机目标分发。
- 退出日志无节点崩溃。

运行日志存放在 `log/runtime_smoke/`，不会提交。RViz 插件测试验证动态库发现和实例构造；鼠标交互及完整 GUI 显示仍需在桌面会话中人工确认。尚未测试真实飞控、Gazebo、CUDA、ROS1 bridge 或其他 ROS2 发行版。

## 来源

来源为本机 FAPP 的 simulation 子树（上游项目：<https://github.com/arclab-hku/FAPP>），此次只修改 `simulation_ros2` 内的副本。各包沿用 package.xml 的原许可类型和已有源码版权声明；具体许可要求以对应文件为准。
