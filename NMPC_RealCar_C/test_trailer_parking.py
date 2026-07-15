import carla
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from nmpc_controller import NMPCController, smooth_path_and_calculate_yaw


def get_transform_behind(transform, distance):
    """
    计算给定 transform 正后方 distance 距离的 transform。
    """
    forward_vector = transform.get_forward_vector()
    new_location = carla.Location(
        x=transform.location.x - (forward_vector.x * distance),
        y=transform.location.y - (forward_vector.y * distance),
        z=transform.location.z - (forward_vector.z * distance),
    )
    new_location.z += 0.02
    return carla.Transform(new_location, transform.rotation)


def normalize_angle(angle):
    """归一化角度到 [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def load_path_from_csv(csv_path):
    """
    从 CSV 文件加载参考路径
    """
    import csv
    points = []
    try:
        # 尝试多种编码方式
        encodings = ['utf-8', 'utf-8-sig', 'gbk', 'gb2312']
        data = None
        
        for encoding in encodings:
            try:
                with open(csv_path, 'r', encoding=encoding) as f:
                    data = f.read()
                break
            except Exception:
                continue
        
        if data is None:
            print("❌ 无法读取文件，所有编码尝试失败")
            return None
        
        # 检查文件内容
        lines = data.strip().split('\n')
        print(f"📄 文件共 {len(lines)} 行")
        if len(lines) > 0:
            print(f"   第一行: {lines[0][:50]}...")
        
        # 使用 csv 模块解析
        import io
        reader = csv.DictReader(io.StringIO(data))
        
        # 检查字段名
        fieldnames = reader.fieldnames
        print(f"📋 检测到字段: {fieldnames}")
        
        for row in reader:
            # 尝试不同的字段名
            x = None
            y = None
            
            for key in row.keys():
                if 'x' in key.lower():
                    x = float(row[key])
                elif 'y' in key.lower():
                    y = float(row[key])
            
            if x is not None and y is not None:
                points.append([x, y])
        
        print(f"✅ 成功从 {csv_path} 加载参考路径：{len(points)} 个点")
        return np.array(points)
        
    except Exception as e:
        import traceback
        print(f"⚠️ 读取路径文件失败：{e}")
        print(f"📝 错误详情：{traceback.format_exc()}")
        return None


def smooth_path(points, window_size=7):
    """
    使用移动平均平滑路径（2D点）
    points: Nx2 numpy array
    window_size: 平滑窗口大小
    """
    points = np.asarray(points)
    if len(points) < window_size:
        return points
    
    n = len(points)
    smoothed = np.zeros_like(points)
    half = window_size // 2
    
    # 首尾部分保持原样
    smoothed[:half] = points[:half]
    smoothed[-half:] = points[-half:]
    
    # 中间部分移动平均
    for i in range(half, n - half):
        smoothed[i] = np.mean(points[i-half:i+half+1], axis=0)
    
    return smoothed


def generate_short_test_path(center_x=0, center_y=0, radius=15, num_points=80):
    """
    生成一段短的测试路径（圆形路径的一部分）
    """
    theta = np.linspace(0, np.pi * 1.2, num_points)
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)
    return np.column_stack([x, y])


def main():
    # ================== 1. 连接 UE5 Carla 服务端 ==================
    client = carla.Client('localhost', 2000)
    client.set_timeout(15.0)
    print("正在连接 Carla 服务器...")
    
    # 获取服务器上已加载的地图（不强制重新加载）
    world = client.get_world()
    map_name = world.get_map().name.split('/')[-1]
    print(f"✅ 成功连接！当前服务器地图：{map_name}")


    blueprint_library = world.get_blueprint_library()
    
    # ================== 启动前的大扫除 ==================
    print("🧹 正在执行启动前大扫除...")
    for a in world.get_actors().filter('*'):
        type_id = a.type_id.lower()
        if 'vehicle' in type_id or 'trailer' in type_id or 'airtor' in type_id:
            if a.is_alive:
                a.destroy()
    print("✨ 清理完毕！")

    # 🌟 开启同步模式的
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    
    print("✅ 成功连接Carla仿真环境！")

    # ================== 2. 加载参考路径 ==================
    print("🗺️ 正在加载参考路径...")
    
    # 从 CSV 文件加载参考路径（已按 yaw 连续性筛选）
    import os
    csv_path = os.path.join(os.path.dirname(__file__), 'user_waypoints.csv')
    print(f"📂 CSV 文件路径：{csv_path}")
    print(f"📁 文件是否存在：{os.path.exists(csv_path)}")
    
    test_path = load_path_from_csv(csv_path)
    
    # 如果加载失败，退出程序
    if test_path is None or len(test_path) < 2:
        print("❌ 无法加载参考路径，请检查文件")
        return
    
    # 打印路径信息（平滑前）
    print(f"📍 原始路径：{len(test_path)} 个点")
    print(f"   路径起点：({test_path[0, 0]:.6f}, {test_path[0, 1]:.6f})")
    print(f"   路径终点：({test_path[-1, 0]:.6f}, {test_path[-1, 1]:.6f})")
    
    # 路径平滑
    print("🔧 正在平滑路径...")
    test_path = smooth_path(test_path, window_size=5)
    print(f"📍 平滑后路径：{len(test_path)} 个点")
    print(f"   路径起点：({test_path[0, 0]:.6f}, {test_path[0, 1]:.6f})")
    print(f"   路径终点：({test_path[-1, 0]:.6f}, {test_path[-1, 1]:.6f})")
    
    # 设置自由视角（路径中间上方俯视）
    spectator = world.get_spectator()
    mid_idx = len(test_path) // 2
    mid_x = test_path[mid_idx, 0]
    mid_y = test_path[mid_idx, 1]
    spectator.set_transform(carla.Transform(
        carla.Location(x=mid_x, y=mid_y - 40, z=60),
        carla.Rotation(pitch=-60, yaw=0, roll=0)
    ))
    
    # ================== 3. 生成车辆（沿用 test.py 的空投+纠偏逻辑） ==================
    actor_list = []
    tractor_bp_id = 'vehicle.xijing.qtractor'
    trailer_bp_ids = [
        'vehicle.qinghua.trailer1',
        'vehicle.qinghua.trailer2',
        'vehicle.qinghua.trailer3',
        'vehicle.qinghua.trailer4',
    ]

    try:
        tractor_bp = blueprint_library.find(tractor_bp_id)
        trailer_bps = [blueprint_library.find(bp_id) for bp_id in trailer_bp_ids]
        print(f"✅ 找到自定义牵引车: {tractor_bp_id}")
    except Exception as e:
        print(f"⚠️ 未找到自定义车型蓝图，回退为 Lincoln MKZ: {e}")
        tractor_bp = blueprint_library.find('vehicle.lincoln.mkz')
        trailer_bps = []

    if len(test_path) < 2:
        raise RuntimeError("❌ 参考路径点不足，无法生成车辆")

    start_point = test_path[0]
    dx = test_path[1][0] - test_path[0][0]
    dy = test_path[1][1] - test_path[0][1]
    init_yaw = math.degrees(math.atan2(dy, dx))

    target_spawn_transform = carla.Transform(
        carla.Location(x=start_point[0], y=start_point[1], z=0.2),
        carla.Rotation(pitch=0.0, yaw=init_yaw, roll=0.0)
    )

    def spawn_actor_with_height_retry(blueprint, base_transform, height_offsets):
        for height_offset in height_offsets:
            test_transform = carla.Transform(
                carla.Location(
                    x=base_transform.location.x,
                    y=base_transform.location.y,
                    z=base_transform.location.z + float(height_offset),
                ),
                base_transform.rotation,
            )
            actor = world.try_spawn_actor(blueprint, test_transform)
            if actor is not None:
                return actor, test_transform
        raise RuntimeError(
            f"Spawn failed because of collision at spawn position: {base_transform.location.x:.2f}, {base_transform.location.y:.2f}, {base_transform.location.z:.2f}"
        )

    print("正在生成牵引车...")
    tractor, tractor_transform = spawn_actor_with_height_retry(
        tractor_bp,
        target_spawn_transform,
        height_offsets=[3.0, 5.0, 8.0, 12.0],
    )
    tractor.set_target_velocity(carla.Vector3D(0, 0, 0))
    tractor.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
    actor_list.append(tractor)

    first_distance = 3.75
    following_distance = 4.85
    current_transform = tractor_transform
    for i, trailer_bp in enumerate(trailer_bps, 1):
        distance = first_distance if i == 1 else following_distance
        print(f"正在生成第{i}节挂车...")
        current_transform = get_transform_behind(current_transform, distance)
        trailer, current_transform = spawn_actor_with_height_retry(
            trailer_bp,
            current_transform,
            height_offsets=[0.0, 0.8, 1.6, 2.4],
        )
        trailer.set_target_velocity(carla.Vector3D(0, 0, 0))
        actor_list.append(trailer)
        time.sleep(0.05)

    print("⏳ 物理预热：等待所有车辆落地并稳定悬挂...")
    for _ in range(40):
        world.tick()

    print("✨ 纠偏传送：强制对齐到绝对完美的挂接坐标...")
    base_transform = tractor.get_transform()
    for i, trailer in enumerate(actor_list[1:], 1):
        distance = first_distance if i == 1 else following_distance
        ideal_transform = get_transform_behind(base_transform, distance)
        ideal_transform.location.z = trailer.get_transform().location.z
        trailer.set_transform(ideal_transform)
        trailer.set_target_velocity(carla.Vector3D(0, 0, 0))
        trailer.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        base_transform = ideal_transform

    print("🔗 位置锁定！等待 UE4 蓝图触发重叠事件...")
    for _ in range(10):
        world.tick()

    print("释放所有制动...")
    for actor in actor_list:
        try:
            if hasattr(actor, 'apply_control'):
                actor.apply_control(carla.VehicleControl(hand_brake=False, brake=0.0))
        except Exception:
            pass

    print("牵引车踩下油门...")
    tractor.set_autopilot(False)
    tractor.apply_control(carla.VehicleControl(throttle=0.2, steer=0.0, hand_brake=False, brake=0.0))

    print("给全体车辆施加唤醒冲量...")
    for actor in actor_list:
        actor.add_impulse(carla.Vector3D(0, 0, 10000))

    vehicle = tractor
    potential_trailers = actor_list[1:]
    num_trailers = len(potential_trailers)
    print(f"✅ 生成完成：牵引车 1 辆，挂车 {num_trailers} 节")

    vehicle_length = vehicle.bounding_box.extent.x * 2
    vehicle_width = vehicle.bounding_box.extent.y * 2
    print(f"📏 车辆尺寸: 长 {vehicle_length:.3f}m, 宽 {vehicle_width:.3f}m")

    trailer_lengths = []
    trailer_widths = []
    for i, tr in enumerate(potential_trailers):
        tr_bbox = tr.bounding_box.extent
        tr_length = tr_bbox.x * 2
        tr_width = tr_bbox.y * 2
        trailer_lengths.append(tr_length)
        trailer_widths.append(tr_width)
        print(f"   挂车 {i+1}: {tr.type_id} - 长 {tr_length:.3f}m, 宽 {tr_width:.3f}m")

    # ================== MATLAB 风格：顺序挂车对齐停靠点 ==================
    # 参考 Park_NMPC_fenduan_SM.m 的分段NMPC逻辑
    # 停靠点设置
    parking_point = np.array([-46.129636,-171.581677], dtype=np.float64)
    alignment_window = 10.0   # 挂车进入此范围后NMPC降速对齐
    stop_approach_zone = 0.45  # 进入此范围后 NMPC target_speed=0 开始减速停车
    stop_speed_threshold = 0.1  # 判定已停稳的速度阈值 (m/s)
    
    # 需要顺序停靠的挂车索引（对应 MATLAB 的 full_trailer_indices）
    target_trailer_indices = list(range(1, num_trailers + 1))  # [1, 2, 3, 4] 所有挂车
    current_target_ptr = 0  # 当前正在对齐第几个目标挂车
    
    print(f"🚧 停靠点: ({parking_point[0]:.3f}, {parking_point[1]:.3f})")
    print(f"📐 顺序挂车对齐：依次对齐挂车 {target_trailer_indices}，共 {len(target_trailer_indices)} 节")

    # ================== 5. 初始化NMPC控制器 ==================
    print("🔧 初始化NMPC控制器...")
    wheelbase = max(2.0, vehicle_length * 0.6)  # 估算轴距
    nmpc = NMPCController(N=25, dt=0.05, wheelbase=wheelbase)
    print(f"   NMPC参数: 预测步数={nmpc.N_p}, 时间步={nmpc.dt}s, 轴距={wheelbase:.2f}m")

    # ================== 6. 初始化可视化 ==================
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    fig.canvas.manager.set_window_title("Trailer Parking Alignment")

    # ================== 7. 主循环 ==================
    print("\n🚀 开始路径跟踪...")
    print("按 Ctrl+C 停止程序")

    # 预热物理引擎
    for _ in range(10):
        world.tick()

    # --- MATLAB风格：顺序挂车状态机 ---
    all_tasks_done = False
    is_stopped = False
    stop_timer = 0
    stop_delay = 1.0  # 停车后等待时间（秒），用于物理稳定
    has_passed_parking = [False] * len(target_trailer_indices)  # 各挂车是否已触发停靠
    skip_until_depart = [False] * len(target_trailer_indices)   # 切目标后先离开停靠区再启用检测
    
    frame_count = 0
    actual_x = []
    actual_y = []
    trailer_history = [[] for _ in range(num_trailers)]
    
    # 记录每节挂车停靠时的位置和误差
    parking_errors = []
    
    try:
        while True:
            frame_count += 1
            
            # 获取车辆状态
            vehicle_transform = vehicle.get_transform()
            current_loc = vehicle_transform.location
            current_yaw_rad = math.radians(vehicle_transform.rotation.yaw)
            velocity = vehicle.get_velocity()
            current_v = math.hypot(velocity.x, velocity.y)

            # 记录轨迹
            actual_x.append(current_loc.x)
            actual_y.append(current_loc.y)
            
            # 记录挂车轨迹
            for i, tr in enumerate(potential_trailers):
                if tr.is_alive:
                    tr_tf = tr.get_transform()
                    trailer_history[i].append([tr_tf.location.x, tr_tf.location.y])

            # # 更新跟随视角 -- 已改为自由视角
            # forward_vec = vehicle_transform.get_forward_vector()
            # spectator_location = vehicle_transform.location - forward_vec * 50.0 + carla.Location(z=50.0)
            # spectator_rotation = carla.Rotation(pitch=-50.0, yaw=vehicle_transform.rotation.yaw, roll=0.0)
            # spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))

            # ================== active_idx 决定逻辑 ==================
            # 参考 Park_NMPC_fenduan_SM.m: 基于指针的顺序锁定
            active_idx = 0
            dist_to_parking = float('inf')
            if not all_tasks_done:
                current_target_trailer_idx = target_trailer_indices[current_target_ptr]
                target_tr = potential_trailers[current_target_trailer_idx - 1]
                if target_tr.is_alive:
                    tr_tf = target_tr.get_transform()
                    tr_center = np.array([tr_tf.location.x, tr_tf.location.y])
                    dist_to_parking = np.linalg.norm(tr_center - parking_point)
                    if dist_to_parking < alignment_window:
                        active_idx = current_target_trailer_idx

            # ================== 状态机 ==================
            throttle, brake, steer = 0.0, 0.0, 0.0  # default

            # 所有挂车停靠完成 → 继续跑完剩下的参考路径，到末端再退出
            if all_tasks_done:
                current_state = [current_loc.x, current_loc.y, current_yaw_rad, current_v]
                throttle, brake, steer = nmpc.solve(current_state, test_path, target_speed=2.5)
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

                last_point = test_path[-1]
                dist_to_end = np.linalg.norm(
                    np.array([current_loc.x, current_loc.y]) - last_point)

                if dist_to_end < 3.0:
                    print("\n🎉 跑完参考路径末端，任务完成！")
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                    for _ in range(20):
                        world.tick()
                    break
                # 不 continue，让后面的可视化和统一 world.tick() 执行

            if is_stopped:
                # 停车保持：打印位姿 + 用户确认 + 切下一节
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                stop_timer += 0.05
                
                if stop_timer >= stop_delay:
                    current_target_trailer_idx = target_trailer_indices[current_target_ptr]
                    tr_tf = potential_trailers[current_target_trailer_idx - 1].get_transform()
                    tr_center_now = np.array([tr_tf.location.x, tr_tf.location.y])
                    tr_yaw_deg = tr_tf.rotation.yaw
                    dist_final = np.linalg.norm(tr_center_now - parking_point)
                    
                    parking_errors.append({
                        'trailer': current_target_trailer_idx,
                        'pos': tr_center_now.copy(),
                        'yaw': tr_yaw_deg,
                        'error_dist': dist_final,
                    })
                    
                    print(f"\n--- 点击确认时的当前位姿 ---")
                    print(f"对应挂车 {current_target_trailer_idx} 位置: ({tr_center_now[0]:.4f}, {tr_center_now[1]:.4f}), 航向: {tr_yaw_deg:.2f}°")
                    print(f"停靠点: ({parking_point[0]:.4f}, {parking_point[1]:.4f})")
                    print(f"距离误差: {dist_final:.4f} m")
                    print(f"----------------------------")
                    
                    if current_target_ptr < len(target_trailer_indices) - 1:
                        current_target_ptr += 1
                        next_trailer = target_trailer_indices[current_target_ptr]
                        print(f"<<< 挂车 {current_target_trailer_idx} 装卸完成，确认起步！")
                        print(f">>> 下一个目标：挂车 {next_trailer}")
                        
                        # 如果新目标挂车已在停靠减速区内，标记先离开再启用
                        next_tr = potential_trailers[next_trailer - 1]
                        if next_tr.is_alive:
                            next_tf = next_tr.get_transform()
                            next_dist = np.linalg.norm(
                                np.array([next_tf.location.x, next_tf.location.y]) - parking_point)
                            if next_dist < stop_approach_zone:
                                has_passed_parking[current_target_ptr] = True
                                skip_until_depart[current_target_ptr] = True
                                print(f"   ⚠️ 挂车 {next_trailer} 已在停车减速区内，等待离开后重新启用...")
                    else:
                        all_tasks_done = True
                        print(f"<<< 挂车 {current_target_trailer_idx} 装卸完成！")
                        print(">>> 所有挂车任务已完成。")
                    
                    # 用户确认后释放制动 + 唤醒冲量
                    print("   按 Enter 键继续...")
                    input()
                    
                    is_stopped = False
                    nmpc.last_u_seq = None
                    
                    # 施加垂直冲量唤醒车辆（Carla重型车队从静止起步需要）
                    for actor in actor_list:
                        if actor.is_alive:
                            actor.add_impulse(carla.Vector3D(0, 0, 5000))
                    
                    # 释放制动，全油门多tick起步
                    vehicle.apply_control(carla.VehicleControl(
                        throttle=1.0, steer=0.0, brake=0.0, hand_brake=False
                    ))
                    for _ in range(30):
                        world.tick()
            else:
                # ================== 正常行驶：NMPC 控制 ==================
                current_state = [current_loc.x, current_loc.y, current_yaw_rad, current_v]
                
                # 如果当前目标挂车需要跳过检测（先离开停靠区）
                if not all_tasks_done and skip_until_depart[current_target_ptr]:
                    if dist_to_parking > stop_approach_zone * 2:
                        skip_until_depart[current_target_ptr] = False
                        has_passed_parking[current_target_ptr] = False
                        print(f"\n✅ 挂车 {target_trailer_indices[current_target_ptr]} 已离开停靠区，重新启用检测")
                    target_speed = 2.5 if active_idx == 0 else 1.39
                elif not all_tasks_done and not has_passed_parking[current_target_ptr] and dist_to_parking < stop_approach_zone:
                    # 目标挂车进入停车减速区 → NMPC target_speed=0 减速到停
                    target_speed = 0.0
                    
                    if current_v < stop_speed_threshold:
                        # 车辆已停稳 → 触发停车记录
                        has_passed_parking[current_target_ptr] = True
                        is_stopped = True
                        stop_timer = 0
                        print(f"\n>>> 挂车 {target_trailer_indices[current_target_ptr]} NMPC减速停车完成！")
                else:
                    # 正常跟踪：对齐区内限速 5km/h(≈1.39m/s)，否则全速 2.5m/s
                    if active_idx > 0:
                        target_speed = 1.39  # 5 km/h
                    else:
                        target_speed = 2.5
                
                throttle, brake, steer = nmpc.solve(current_state, test_path, target_speed=target_speed)
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steer, brake=brake))

            # ================== 打印状态信息 ==================
            speed_kmh = 3.6 * current_v
            if all_tasks_done:
                last_point = test_path[-1]
                dist_to_end = np.linalg.norm(
                    np.array([current_loc.x, current_loc.y]) - last_point)
                status = f"跑完剩余路径(距末端:{dist_to_end:.1f}m)"
            elif is_stopped:
                status = f"停车中(挂车{target_trailer_indices[current_target_ptr]})"
            elif active_idx > 0:
                status = f"对齐挂车{active_idx}(距离:{dist_to_parking:.1f}m)"
            else:
                status = "牵引车跟踪"
            print(
                f"\r{status} | 速度: {speed_kmh:>4.1f}km/h "
                f"| 油门: {throttle:>4.2f} | 刹车: {brake:>4.2f} | 转向: {math.degrees(steer * nmpc.max_steer):>6.1f}°",
                end="",
                flush=True
            )

            # ================== 更新可视化 ==================
            if frame_count % 3 == 0:
                # 左图：俯视图
                ax1.cla()
                
                # 参考路径
                ax1.plot(test_path[:, 0], test_path[:, 1], 'g--', linewidth=2, label='Reference Path')
                
                # 高亮停靠点（大红色星号）
                ax1.plot(parking_point[0], parking_point[1], 'r*', markersize=15, label='Parking Point')
                # 对齐窗口（大圈）和停车减速区（小圈）
                theta_circle = np.linspace(0, 2 * np.pi, 60)
                ax1.plot(parking_point[0] + alignment_window * np.cos(theta_circle),
                         parking_point[1] + alignment_window * np.sin(theta_circle),
                         'g--', linewidth=1, alpha=0.4, label=f'Alignment ({alignment_window}m)')
                ax1.plot(parking_point[0] + stop_approach_zone * np.cos(theta_circle),
                         parking_point[1] + stop_approach_zone * np.sin(theta_circle),
                         'r--', linewidth=1, alpha=0.5, label=f'Stop Zone ({stop_approach_zone}m)')
                
                # 车辆轨迹
                ax1.plot(actual_x, actual_y, 'b-', linewidth=2, label='Tractor Path')
                
                # 挂车轨迹
                colors = ['r-', 'm-', 'c-', 'y-', 'k-']
                for i, hist in enumerate(trailer_history):
                    if len(hist) > 1:
                        hist_arr = np.array(hist)
                        ax1.plot(hist_arr[:, 0], hist_arr[:, 1], colors[i % len(colors)], linewidth=1.5, 
                                label=f'Trailer {i+1} Path')
                
                # 当前车辆位置
                ax1.plot(current_loc.x, current_loc.y, 'bo', markersize=10, label='Tractor')
                
                # 挂车当前位置
                for i, tr in enumerate(potential_trailers):
                    if tr.is_alive:
                        tr_tf = tr.get_transform()
                        ax1.plot(tr_tf.location.x, tr_tf.location.y, colors[i % len(colors)][0] + 'o', markersize=8)
                
                ax1.axis('equal')
                if all_tasks_done:
                    ax1.set_title("Completing Reference Path - All Trailers Parked")
                elif active_idx > 0:
                    ax1.set_title(f"Sequential Trailer Alignment - Aligning Trailer {active_idx}")
                else:
                    ax1.set_title("Tractor Path Tracking")
                ax1.legend(loc='upper right')
                ax1.set_xlabel('X (m)')
                ax1.set_ylabel('Y (m)')
                
                # 右图：停靠误差统计
                ax2.cla()
                if len(parking_errors) > 0:
                    trailer_nums = [pe['trailer'] for pe in parking_errors]
                    errors = [pe['error_dist'] for pe in parking_errors]
                    colors_bar = ['b', 'm', 'c', 'y'][:len(errors)]
                    ax2.bar(trailer_nums, errors, color=colors_bar, width=0.4)
                    ax2.set_xlabel('Trailer Index')
                    ax2.set_ylabel('Distance Error (m)')
                    ax2.set_title('Parking Alignment Error per Trailer')
                    ax2.set_xticks(trailer_nums)
                    ax2.grid(True, alpha=0.3, axis='y')
                    # 标注数值
                    for i, (tn, err) in enumerate(zip(trailer_nums, errors)):
                        ax2.text(tn, err + 0.05, f'{err:.3f}', ha='center', fontsize=9)
                
                plt.pause(0.001)

            # 步进仿真
            world.tick()

    except KeyboardInterrupt:
        print("\n🛑 程序被手动停止")

    finally:
        # 清理环境
        print("\n🧹 正在清理仿真环境...")
        
        # 关闭同步模式
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        
        # 删除所有相关Actor
        for a in world.get_actors().filter('*'):
            type_id = a.type_id.lower()
            if 'vehicle' in type_id or 'trailer' in type_id or 'airtor' in type_id:
                if a.is_alive:
                    a.destroy()
        
        plt.close('all')
        
        # 打印最终停靠误差统计
        if parking_errors:
            print("\n📊 挂车停靠误差统计:")
            for pe in parking_errors:
                print(f"   挂车 {pe['trailer']}: 位置 ({pe['pos'][0]:.4f}, {pe['pos'][1]:.4f}), "
                      f"航向 {pe['yaw']:.2f}°, 误差 {pe['error_dist']:.4f} m")
            print(f"   停靠点: ({parking_point[0]:.4f}, {parking_point[1]:.4f})")
        
        print("✅ 环境清理完成！")


if __name__ == '__main__':
    main()