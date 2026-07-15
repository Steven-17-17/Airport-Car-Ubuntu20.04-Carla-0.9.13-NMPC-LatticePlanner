#!/usr/bin/env python3

import csv
import math
import os
import sys
import time

import carla


def make_pose_from_transform(transform):
    from geometry_msgs.msg import Pose

    yaw = math.radians(transform.rotation.yaw)
    half_yaw = yaw * 0.5

    pose = Pose()
    pose.position.x = transform.location.x
    pose.position.y = transform.location.y
    pose.position.z = transform.location.z
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(half_yaw)
    pose.orientation.w = math.cos(half_yaw)
    return pose


def yaw_from_transform(transform):
    return math.radians(transform.rotation.yaw)


def publish_ros_state_until_shutdown(actor_list):
    try:
        import rospy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseArray
        from std_msgs.msg import Float64MultiArray
    except Exception as exception:
        print(f"未启用 ROS 状态发布，原因: {exception}")
        return

    rospy.init_node("nmpc_trailer_pose_publisher", anonymous=True, disable_signals=True)
    odom_pub = rospy.Publisher("/carla/ego_vehicle/odometry", Odometry, queue_size=1)
    pose_pub = rospy.Publisher("/trailers/poses", PoseArray, queue_size=1)
    state_pub = rospy.Publisher("/trailers/states", Float64MultiArray, queue_size=1)
    rate = rospy.Rate(20.0)
    tractor = actor_list[0]
    trailers = actor_list[1:]

    print("开始发布 /carla/ego_vehicle/odometry、/trailers/poses 和 /trailers/states。按 Ctrl+C 或停止 roslaunch 结束。")
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()

        if tractor.is_alive:
            tractor_transform = tractor.get_transform()
            tractor_velocity = tractor.get_velocity()
            tractor_angular_velocity = tractor.get_angular_velocity()

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "map"
            odom.child_frame_id = "ego_vehicle"
            odom.pose.pose = make_pose_from_transform(tractor_transform)
            odom.twist.twist.linear.x = tractor_velocity.x
            odom.twist.twist.linear.y = tractor_velocity.y
            odom.twist.twist.linear.z = tractor_velocity.z
            odom.twist.twist.angular.x = math.radians(tractor_angular_velocity.x)
            odom.twist.twist.angular.y = math.radians(tractor_angular_velocity.y)
            odom.twist.twist.angular.z = math.radians(tractor_angular_velocity.z)
            odom_pub.publish(odom)

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = "map"
        trailer_states = Float64MultiArray()

        for index, trailer in enumerate(trailers, 1):
            if trailer.is_alive:
                trailer_transform = trailer.get_transform()
                trailer_yaw_rad = yaw_from_transform(trailer_transform)
                pose_array.poses.append(make_pose_from_transform(trailer_transform))
                trailer_states.data.extend([
                    float(index),
                    trailer_transform.location.x,
                    trailer_transform.location.y,
                    trailer_transform.location.z,
                    trailer_yaw_rad,
                    math.degrees(trailer_yaw_rad),
                ])

        pose_pub.publish(pose_array)
        state_pub.publish(trailer_states)
        rate.sleep()


def load_reference_points(csv_path):
    encodings = ["utf-8", "utf-8-sig", "gbk", "gb2312"]
    data = None

    for encoding in encodings:
        try:
            with open(csv_path, "r", encoding=encoding) as file_handle:
                data = file_handle.read()
            break
        except Exception:
            continue

    if data is None:
        raise RuntimeError(f"无法读取参考路径文件: {csv_path}")

    points = []
    reader = csv.DictReader(data.splitlines())
    for row in reader:
        x_value = None
        y_value = None
        for key, value in row.items():
            if key is None:
                continue
            key_lower = key.lower()
            if x_value is None and "x" in key_lower:
                x_value = float(value)
            elif y_value is None and "y" in key_lower:
                y_value = float(value)
        if x_value is not None and y_value is not None:
            points.append((x_value, y_value))

    if len(points) < 2:
        raise RuntimeError("参考路径点不足，无法生成车辆")

    return points


def get_transform_behind(transform, distance):
    forward_vector = transform.get_forward_vector()
    new_location = carla.Location(
        x=transform.location.x - (forward_vector.x * distance),
        y=transform.location.y - (forward_vector.y * distance),
        z=transform.location.z - (forward_vector.z * distance),
    )
    new_location.z += 0.02
    return carla.Transform(new_location, transform.rotation)


def spawn_actor_with_height_retry(world, blueprint, base_transform, height_offsets):
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
        "Spawn failed at position: "
        f"{base_transform.location.x:.2f}, {base_transform.location.y:.2f}, {base_transform.location.z:.2f}"
    )


def main():
    root_dir = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = os.path.join(root_dir, "user_waypoints.csv")

    client = carla.Client("localhost", 2000)
    client.set_timeout(15.0)

    print("正在等待 CARLA 世界就绪...")
    world = None
    for _ in range(60):
        try:
            world = client.get_world()
            if world is not None and world.get_map() is not None:
                break
        except Exception:
            pass
        time.sleep(1.0)

    if world is None:
        raise RuntimeError("无法连接到已启动的 CARLA 世界")

    blueprint_library = world.get_blueprint_library()
    print(f"当前地图: {world.get_map().name}")

    print("正在清理场景中的旧车辆...")
    for actor in world.get_actors().filter("*"):
        type_id = actor.type_id.lower()
        if "vehicle" in type_id or "trailer" in type_id or "airtor" in type_id:
            if actor.is_alive:
                actor.destroy()

    reference_points = load_reference_points(csv_path)
    start_x, start_y = reference_points[0]
    next_x, next_y = reference_points[1]
    init_yaw = math.degrees(math.atan2(next_y - start_y, next_x - start_x))

    tractor_bp_id = "vehicle.xijing.qtractor"
    trailer_bp_ids = [
        "vehicle.qinghua.trailer1",
        "vehicle.qinghua.trailer2",
        "vehicle.qinghua.trailer3",
        "vehicle.qinghua.trailer4",
    ]

    try:
        tractor_bp = blueprint_library.find(tractor_bp_id)
        trailer_bps = [blueprint_library.find(bp_id) for bp_id in trailer_bp_ids]
        print(f"使用自定义牵引车: {tractor_bp_id}")
    except Exception as exception:
        print(f"未找到自定义车型，回退到 Lincoln MKZ: {exception}")
        tractor_bp = blueprint_library.find("vehicle.lincoln.mkz")
        trailer_bps = []

    if tractor_bp.has_attribute("role_name"):
        tractor_bp.set_attribute("role_name", "ego_vehicle")
    for index, trailer_bp in enumerate(trailer_bps, 1):
        if trailer_bp.has_attribute("role_name"):
            trailer_bp.set_attribute("role_name", f"trailer_{index}")

    target_spawn_transform = carla.Transform(
        carla.Location(x=start_x, y=start_y, z=0.2),
        carla.Rotation(pitch=0.0, yaw=init_yaw, roll=0.0),
    )

    actor_list = []
    first_distance = 3.75
    following_distance = 4.85

    print("正在生成牵引车和挂车...")
    tractor, tractor_transform = spawn_actor_with_height_retry(
        world,
        tractor_bp,
        target_spawn_transform,
        height_offsets=[3.0, 5.0, 8.0, 12.0],
    )
    tractor.set_target_velocity(carla.Vector3D(0, 0, 0))
    tractor.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
    actor_list.append(tractor)

    current_transform = tractor_transform
    for index, trailer_bp in enumerate(trailer_bps, 1):
        distance = first_distance if index == 1 else following_distance
        current_transform = get_transform_behind(current_transform, distance)
        trailer, current_transform = spawn_actor_with_height_retry(
            world,
            trailer_bp,
            current_transform,
            height_offsets=[0.0, 0.8, 1.6, 2.4],
        )
        trailer.set_target_velocity(carla.Vector3D(0, 0, 0))
        actor_list.append(trailer)
        time.sleep(0.05)

    print("进行挂接纠偏...")
    base_transform = tractor.get_transform()
    for index, trailer in enumerate(actor_list[1:], 1):
        distance = first_distance if index == 1 else following_distance
        ideal_transform = get_transform_behind(base_transform, distance)
        ideal_transform.location.z = trailer.get_transform().location.z
        trailer.set_transform(ideal_transform)
        trailer.set_target_velocity(carla.Vector3D(0, 0, 0))
        trailer.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        base_transform = ideal_transform

    print("释放制动并给车辆一个向上冲量，唤醒整列车队...")
    for actor in actor_list:
        try:
            if hasattr(actor, "apply_control"):
                actor.apply_control(carla.VehicleControl(hand_brake=False, brake=0.0))
        except Exception:
            pass

    tractor.set_autopilot(False)
    tractor.apply_control(carla.VehicleControl(throttle=0.2, steer=0.0, hand_brake=False, brake=0.0))

    for actor in actor_list:
        try:
            actor.add_impulse(carla.Vector3D(0, 0, 5000))
        except Exception:
            pass

    time.sleep(0.5)

    print("原始地图和车辆已就绪。")
    publish_ros_state_until_shutdown(actor_list)
    print("场景已准备完成，启动器退出，但车辆会保留在服务器里。")


if __name__ == "__main__":
    main()
