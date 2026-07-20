# NMPC_RealCar_CPP

这是一个用于 **CARLA 0.9.13-dirty + ROS1 Noetic** 的四节全挂车路径跟踪与顺序停靠控制工程。控制核心已经迁移为 C++，ROS 包名为：

```bash
nmpc_realcar_cpp
```

工程会读取 `user_waypoints.csv` 作为参考路径，在 CARLA 中生成牵引车和 4 节挂车，并通过 C++ 控制节点发布车辆控制命令。

## 1. GitHub 需要包含什么

建议上传这些源码和配置文件：

```text
NMPC_RealCar_C/
  CMakeLists.txt
  package.xml
  README.md
  user_waypoints.csv
  test_trailer_parking.cpp
  nmpc_controller.py
  test_trailer_parking.py
  launch_original_scene.py
  include/nmpc_realcar_cpp/test_trailer_parking.h
  src/ros_trailer_parking_node.cpp
  launch/trailer_parking.launch
```

不建议上传这些生成文件：

```text
build/
devel/
install/
__pycache__/
*.pyc
test_trailer_parking
```

注意：这个工程依赖你的 **CARLA 0.9.13-dirty 自建版本**。如果自建地图和自定义车辆蓝图体积很大，通常不要直接放进这个 ROS 包里；建议在 README 中说明 CARLA 包的获取方式，或者单独用 Release/网盘/内部仓库管理。

## 2. 环境要求

推荐系统：

- Ubuntu 20.04
- ROS1 Noetic
- Python 3
- CARLA `0.9.13-dirty`
- 已包含自建地图和自定义车辆蓝图的 CARLA 包

需要的 ROS 依赖：

- `roscpp`
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `carla_msgs`
- `carla_ros_bridge`

需要的编译工具：

- `catkin`
- `cmake`
- `g++`

默认 CARLA 地图：

```bash
map_dxf/Maps/dxf_map/dxf_map
```

默认自定义车辆蓝图：

```text
vehicle.xijing.qtractor
vehicle.qinghua.trailer1
vehicle.qinghua.trailer2
vehicle.qinghua.trailer3
vehicle.qinghua.trailer4
```

如果新电脑上的 CARLA 包没有这些地图或车辆蓝图，`launch_original_scene.py` 无法生成当前场景。

## 3. 第一次在新电脑运行

### 3.1 准备工作空间

推荐工作空间结构如下。`steven` 只是作者电脑用户名，新电脑请换成自己的用户名路径。

```text
~/catkin_ws
  └── src
      ├── NMPC_RealCar_C
      └── ros-bridge
```

其中：

- `NMPC_RealCar_C`：本工程目录，虽然目录名保留了 C，但 ROS 包名已经是 `nmpc_realcar_cpp`
- `ros-bridge`：CARLA ROS bridge 源码

### 3.2 编译

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --cmake-args -DBUILD_ROS1=ON
source devel/setup.bash
```

确认包名可被 ROS 找到：

```bash
rospack find nmpc_realcar_cpp
```

编译成功后，控制节点位于：

```bash
~/catkin_ws/devel/lib/nmpc_realcar_cpp/ros_trailer_parking_node
```

### 3.3 检查 CSV

参考路径文件：

```bash
~/catkin_ws/src/NMPC_RealCar_C/user_waypoints.csv
```

CSV 至少需要包含 `x,y` 两列：

```csv
x,y
108.082539,-363.216358
107.068772,-363.176888
```

独立检查路径读取和 C++ 控制核心初始化：

```bash
cd ~/catkin_ws/src/NMPC_RealCar_C
cmake -S . -B /tmp/nmpc_realcar_cpp_build
cmake --build /tmp/nmpc_realcar_cpp_build -j
/tmp/nmpc_realcar_cpp_build/test_trailer_parking user_waypoints.csv
```

## 4. 启动方式

推荐使用 3 个终端。

### 终端 1：启动 CARLA

进入 CARLA 安装目录：

```bash
./CarlaUE4.sh
```

等待 CARLA 完全启动。

### 终端 2：启动 ROS master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 终端 3：启动本工程

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch nmpc_realcar_cpp trailer_parking.launch
```

如果要指定 CSV：

```bash
roslaunch nmpc_realcar_cpp trailer_parking.launch \
  csv_path:=$HOME/catkin_ws/src/NMPC_RealCar_C/user_waypoints.csv
```

如果想强制控制节点等到挂车状态后才发布控制：

```bash
roslaunch nmpc_realcar_cpp trailer_parking.launch require_trailers:=true
```

## 5. `launch_original_scene.py` 是做什么的

`launch_original_scene.py` **不是** 用来启动 CARLA 本体的。CARLA 仍然需要先手动运行：

```bash
./CarlaUE4.sh
```

这个脚本只在 CARLA 已经启动后工作，作用是：

- 连接 CARLA server
- 清理旧车辆
- 根据 `user_waypoints.csv` 起点和航向生成牵引车
- 在牵引车后方生成 4 节挂车
- 发布 `/carla/ego_vehicle/odometry`
- 发布 `/trailers/poses`
- 发布 `/trailers/states`

所以它是 CARLA 仿真场景适配脚本。上实际车辆时，一般不再使用这个脚本。

## 6. 控制逻辑说明

C++ 控制核心：

```text
test_trailer_parking.cpp
include/nmpc_realcar_cpp/test_trailer_parking.h
```

主要逻辑：

- 读取 `user_waypoints.csv`
- 平滑并重采样参考路径
- 计算参考路径 yaw
- 使用单车运动学模型预测
- 在 25 步预测时域内优化 `[delta, v]` 控制序列
- 使用上一帧控制序列热启动
- 按 1 到 4 节挂车顺序停靠
- 进入停靠点附近后降速
- 目标挂车进入停车区后停车
- 收到 `/nmpc/release` 后切换到下一节挂车

当前 C++ 版没有直接调用 SciPy/SLSQP，而是使用无外部依赖的约束坐标搜索近似求解 NMPC 序列优化。

## 7. 常用 Topic

### 7.1 输入/状态 Topic

牵引车里程计：

```bash
rostopic echo /carla/ego_vehicle/odometry
```

四节挂车位姿：

```bash
rostopic echo /trailers/poses
```

四节挂车更直观的状态数组：

```bash
rostopic echo /trailers/states
```

`/trailers/states` 数据格式按 6 个数一组重复：

```text
[编号, x, y, z, yaw_rad, yaw_deg]
```

停车后放行下一节挂车：

```bash
rostopic pub /nmpc/release std_msgs/Bool "data: true" -1
```

### 7.2 控制输出 Topic

发给 CARLA 的控制命令：

```bash
rostopic echo /carla/ego_vehicle/vehicle_control_cmd
```

字段：

- `throttle`：油门，范围 0 到 1
- `brake`：刹车，范围 0 到 1
- `steer`：归一化转向，范围 -1 到 1

牵引车速度：

```bash
rostopic echo /nmpc/tractor_speed
```

前轮转角，单位 rad：

```bash
rostopic echo /nmpc/front_steer_angle
```

控制节点内部挂车状态：

```bash
rostopic echo /nmpc/trailers/states
```

数据格式按 5 个数一组重复：

```text
[编号, x, y, yaw_rad, yaw_deg]
```

控制决策耗时，单位 ms：

```bash
rostopic echo /nmpc/decision_time_ms
```

控制调试数组：

```bash
rostopic echo /nmpc/control_debug
```

数据格式：

```text
[target_speed,
 throttle,
 brake,
 steer_norm,
 steer_rad,
 decision_time_ms,
 active_trailer,
 current_target_trailer,
 is_stopped,
 nmpc_cost]
```

## 8. 上实际车辆需要改什么

建议保留 C++ 控制核心：

```text
test_trailer_parking.cpp
include/nmpc_realcar_cpp/test_trailer_parking.h
```

需要替换的是仿真接口层。当前 CARLA 链路是：

```text
launch_original_scene.py / carla_ros_bridge
  -> /carla/ego_vehicle/odometry
  -> /trailers/poses
  -> ros_trailer_parking_node
  -> /carla/ego_vehicle/vehicle_control_cmd
```

上实车时应改成：

```text
实车定位/传感器/挂车测量
  -> 车辆状态
  -> 挂车状态
  -> C++ 控制节点
  -> 实车底盘控制接口
```

需要对接的输入：

- 牵引车位置 `x, y`
- 牵引车航向角 `yaw`
- 牵引车速度 `v`
- 4 节挂车位置 `x, y`
- 4 节挂车航向角 `yaw`
- 放行/继续信号，对应当前 `/nmpc/release`

需要对接的输出：

- 驱动或目标速度命令
- 制动命令
- 转向命令

如果实车底盘不是 CARLA 的油门/刹车/归一化转向接口，需要修改：

```text
src/ros_trailer_parking_node.cpp
```

当前控制核心输出：

```text
ControlCommand.throttle
ControlCommand.brake
ControlCommand.steer
```

其中 `steer` 是归一化转向，范围 `[-1, 1]`。如果实车需要前轮转角：

```text
front_steer_rad = steer * max_steer_rad
```

实车迁移重点检查：

- 坐标系是否统一
- yaw 正方向是否一致
- 单位是否统一为 m、m/s、rad
- 转向符号是否需要取反
- `max_steer_rad_` 是否符合实车最大前轮转角
- 控制周期是否保持在期望频率
- 是否加入急停、通信超时刹车、人工接管、速度限制

## 9. 常用检查命令

查看 topic：

```bash
rostopic list
```

查看频率：

```bash
rostopic hz /carla/ego_vehicle/odometry
rostopic hz /trailers/states
rostopic hz /carla/ego_vehicle/vehicle_control_cmd
```

查看节点：

```bash
rosnode list
rosnode info /ros_trailer_parking_node
```

确认决策是否运行：

```bash
rostopic echo /nmpc/decision_time_ms
```

## 10. 常见问题

### 找不到包 `nmpc_realcar_cpp`

先重新编译并 source：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --cmake-args -DBUILD_ROS1=ON
source devel/setup.bash
rospack find nmpc_realcar_cpp
```

### 没有 `/carla/ego_vehicle/odometry`

本工程由 `launch_original_scene.py` 主动发布 `/carla/ego_vehicle/odometry`。如果没有消息，检查：

```bash
rosnode list
rostopic list
```

并确认脚本没有因为 CARLA 连接失败、地图错误或找不到车辆蓝图而退出。

### 有 odometry，但是没有控制输出

检查控制节点是否启动：

```bash
rosnode info /ros_trailer_parking_node
rostopic echo /carla/ego_vehicle/vehicle_control_cmd
```

如果启动时设置了 `require_trailers:=true`，还要确认：

```bash
rostopic echo /trailers/poses
```

### VS Code 里 ROS 头文件标红

如果 `catkin_make` 能通过，但 VS Code 标红，一般是 IntelliSense 没有 ROS include 路径。建议从已 source 的终端启动 VS Code：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
code .
```

或者在 `.vscode/c_cpp_properties.json` 中加入：

```text
/opt/ros/noetic/include
~/catkin_ws/devel/include
~/catkin_ws/src/NMPC_RealCar_C/include
```

## 11. 文件说明

- `test_trailer_parking.cpp`：C++ 控制核心，包含路径处理、NMPC 近似求解、停车状态机。
- `include/nmpc_realcar_cpp/test_trailer_parking.h`：C++ 控制核心头文件。
- `src/ros_trailer_parking_node.cpp`：ROS1 控制节点，订阅状态并发布控制命令。
- `launch_original_scene.py`：CARLA 场景适配脚本，生成牵引车/挂车并发布仿真状态。
- `launch/trailer_parking.launch`：一键启动 launch。
- `user_waypoints.csv`：用户参考路径。
- `nmpc_controller.py`：原 Python NMPC 控制器参考实现。
- `test_trailer_parking.py`：原 Python/CARLA 联调参考脚本。
