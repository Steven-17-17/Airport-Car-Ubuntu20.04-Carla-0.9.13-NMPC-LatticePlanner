# NMPC_RealCar_C

这是一个用于 **CARLA 0.9.13-dirty + ROS1 Noetic** 的四节全挂车路径跟踪与顺序停靠控制工程。

工程会根据 `user_waypoints.csv` 生成参考路径，在 CARLA 中生成牵引车和 4 节挂车，通过 ROS topic 发布车辆/挂车状态，并由 C/C++ 控制节点发布 CARLA 控制命令。

## 1. 环境要求

推荐系统：

- Ubuntu 20.04
- ROS1 Noetic
- Python 3
- CARLA `0.9.13-dirty`
- 已包含自建地图和自定义车辆蓝图的 CARLA 包

需要的 ROS 包：

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
- `gcc/g++`

本工程默认使用的 CARLA 地图参数为：

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

如果新电脑上的 CARLA 包没有这些地图或车辆蓝图，场景脚本无法按当前配置生成原始车辆队列。

## 2. 第一次在新电脑运行

### 2.1 安装 ROS Noetic

确认 ROS 可用：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

如果能正常启动 `roscore`，说明 ROS 基础环境可用。

### 2.2 准备 CARLA 0.9.13-dirty

进入 CARLA 根目录，确认可以启动：

```bash
./CarlaUE4.sh
```

启动后确认地图和自定义车辆蓝图已经包含在当前 CARLA 包里。这个工程依赖你的自建地图和自定义车辆，不建议换成官方原版 CARLA 后直接运行。

### 2.3 准备 ROS bridge 工作空间（注意！！！steven是作者个人电脑名字）

工作空间推荐结构：

```text
/home/steven/catkin_ws
  └── src
      ├── NMPC_RealCar_C
      └── ros-bridge
```

如果是新电脑，先把 `carla_ros_bridge` 放到 `catkin_ws/src/` 下，并确保 `carla_msgs` 能被 catkin 找到。

然后编译：

```bash
cd /home/steven/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --cmake-args -DBUILD_ROS1=ON
source devel/setup.bash
```

编译成功后，控制节点会生成在：

```bash
/home/steven/catkin_ws/devel/lib/nmpc_realcar_c/ros_trailer_parking_node
```

### 2.4 检查 CSV 路径

参考路径文件为：

```bash
/home/steven/catkin_ws/src/NMPC_RealCar_C/user_waypoints.csv
```

CSV 至少需要包含 `x,y` 两列，例如：

```csv
x,y
108.082539,-363.216358
107.068772,-363.176888
```

独立检查 CSV 是否能读取：

```bash
cd /home/steven/catkin_ws/src/NMPC_RealCar_C
cmake -S . -B /tmp/nmpc_realcar_c_build
cmake --build /tmp/nmpc_realcar_c_build -j
/tmp/nmpc_realcar_c_build/test_trailer_parking user_waypoints.csv
```

## 3. 启动方式

推荐使用 3 个终端。

### 终端 1：启动 CARLA

进入 CARLA 安装目录：

```bash
./CarlaUE4.sh
```

等待 CARLA 完全启动并进入地图。

### 终端 2：启动 ROS master

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 终端 3：启动本工程

```bash
cd /home/steven/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch nmpc_realcar_c trailer_parking.launch
```

这个 launch 会做几件事：

- 启动 `carla_ros_bridge`
- 加载/连接 CARLA 地图
- 运行 `launch_original_scene.py`
- 清理旧车辆
- 按 `user_waypoints.csv` 起点生成牵引车和 4 节挂车
- 发布牵引车 odometry 和挂车状态
- 启动 `ros_trailer_parking_node`
- 发布 `/carla/ego_vehicle/vehicle_control_cmd` 控制车辆

如果需要手动指定 CSV：

```bash
roslaunch nmpc_realcar_c trailer_parking.launch \
  csv_path:=/home/steven/catkin_ws/src/NMPC_RealCar_C/user_waypoints.csv
```

如果想强制等到挂车状态后才发布控制：

```bash
roslaunch nmpc_realcar_c trailer_parking.launch require_trailers:=true
```

## 4. 控制逻辑说明

C 控制核心位于：

```bash
src/NMPC_RealCar_C/test_trailer_parking.c
```

它按 Python 版 `test_trailer_parking.py` / `nmpc_controller.py` 的主要逻辑迁移：

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

当前 C 版没有直接调用 SciPy/SLSQP，而是使用无外部依赖的约束坐标搜索近似求解 NMPC 序列优化。

## 5. 常用 Topic

### 5.1 输入/状态 Topic

牵引车里程计：

```bash
rostopic echo /carla/ego_vehicle/odometry
```

说明：

- `pose.pose.position.x/y/z`：牵引车位置
- `pose.pose.orientation`：牵引车四元数姿态
- `twist.twist.linear.x/y/z`：牵引车线速度
- `twist.twist.angular.x/y/z`：牵引车角速度

四节挂车位姿：

```bash
rostopic echo /trailers/poses
```

说明：

- `poses[0]`：第 1 节挂车
- `poses[1]`：第 2 节挂车
- `poses[2]`：第 3 节挂车
- `poses[3]`：第 4 节挂车

四节挂车更直观的状态数组：

```bash
rostopic echo /trailers/states
```

数据格式按 6 个数一组重复：

```text
[编号, x, y, z, yaw_rad, yaw_deg]
```

例如：

```text
[1, x1, y1, z1, yaw1_rad, yaw1_deg,
 2, x2, y2, z2, yaw2_rad, yaw2_deg,
 3, x3, y3, z3, yaw3_rad, yaw3_deg,
 4, x4, y4, z4, yaw4_rad, yaw4_deg]
```

停车后放行下一节挂车：

```bash
rostopic pub /nmpc/release std_msgs/Bool "data: true" -1
```

### 5.2 控制输出 Topic

发给 CARLA 的控制命令：

```bash
rostopic echo /carla/ego_vehicle/vehicle_control_cmd
```

字段说明：

- `throttle`：油门，范围 0 到 1
- `brake`：刹车，范围 0 到 1
- `steer`：归一化转向，范围 -1 到 1

牵引车速度：

```bash
rostopic echo /nmpc/tractor_speed
```

前轮转角：

```bash
rostopic echo /nmpc/front_steer_angle
```

单位为 rad。

控制节点内部挂车状态：

```bash
rostopic echo /nmpc/trailers/states
```

数据格式按 5 个数一组重复：

```text
[编号, x, y, yaw_rad, yaw_deg]
```

控制决策耗时：

```bash
rostopic echo /nmpc/decision_time_ms
```

单位为 ms。

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

## 6. 常用检查命令

查看当前 topic：

```bash
rostopic list
```

查看 topic 频率：

```bash
rostopic hz /carla/ego_vehicle/odometry
rostopic hz /trailers/states
rostopic hz /carla/ego_vehicle/vehicle_control_cmd
```

查看节点：

```bash
rosnode list
```

查看节点订阅和发布：

```bash
rosnode info /ros_trailer_parking_node
```

确认车辆是否收到控制：

```bash
rostopic echo /carla/ego_vehicle/vehicle_control_cmd
```

确认决策是否运行：

```bash
rostopic echo /nmpc/decision_time_ms
```

## 7. 常见问题

### 没有 `/carla/ego_vehicle/odometry`

本工程不依赖 `carla_spawn_objects` 的 odometry pseudo sensor，而是由 `launch_original_scene.py` 主动发布 `/carla/ego_vehicle/odometry`。

如果没有消息，优先检查：

```bash
rosnode list
rostopic list
```

并确认 `launch_original_scene.py` 没有因为 CARLA 连接失败或找不到车辆蓝图而退出。

### 有 odometry，但是没有控制输出

检查控制节点是否启动：

```bash
rosnode list
rosnode info /ros_trailer_parking_node
```

检查控制输出：

```bash
rostopic echo /carla/ego_vehicle/vehicle_control_cmd
```

如果启动时设置了 `require_trailers:=true`，还需要确认：

```bash
rostopic echo /trailers/poses
```

### 车辆生成失败

常见原因：

- CARLA 版本不是当前自建的 `0.9.13-dirty`
- 自建地图没有加载
- 自定义车辆蓝图不存在
- 起点附近碰撞导致 spawn 失败

优先检查 launch 终端里的错误输出。

### 车辆能动但方向不对

检查：

```bash
rostopic echo /carla/ego_vehicle/odometry
rostopic echo /nmpc/control_debug
```

重点看车辆 yaw、`steer_norm` 和 `steer_rad` 是否符合预期。

## 8. 文件说明

- `test_trailer_parking.c`：C 控制核心，包含路径处理、NMPC 近似求解、停车状态机。
- `include/nmpc_realcar_c/test_trailer_parking.h`：C 控制核心头文件。
- `src/ros_trailer_parking_node.cpp`：ROS1 控制节点，订阅状态并发布 CARLA 控制命令。
- `launch_original_scene.py`：连接 CARLA，生成牵引车/挂车，发布 odometry 和挂车状态。
- `launch/trailer_parking.launch`：一键启动 launch。
- `user_waypoints.csv`：用户参考路径。
- `nmpc_controller.py`：原 Python NMPC 控制器参考实现。
- `test_trailer_parking.py`：原 Python/CARLA 联调参考脚本。
