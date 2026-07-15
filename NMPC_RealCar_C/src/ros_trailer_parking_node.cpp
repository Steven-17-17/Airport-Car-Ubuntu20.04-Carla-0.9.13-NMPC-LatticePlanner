#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <carla_msgs/CarlaEgoVehicleControl.h>

#include "nmpc_realcar_c/test_trailer_parking.h"

class RosTrailerParkingNode {
public:
    RosTrailerParkingNode()
        : pnh_("~"), have_vehicle_(false), have_trailers_(false), release_requested_(false) {
        pnh_.param<std::string>("csv_path", csv_path_, std::string("user_waypoints.csv"));
        pnh_.param<double>("wheelbase", wheelbase_, 2.8);
        pnh_.param<double>("dt", dt_, 0.05);
        pnh_.param<double>("rate_hz", rate_hz_, 20.0);
        pnh_.param<bool>("require_trailers", require_trailers_, false);
        pnh_.param<std::string>("vehicle_odom_topic", vehicle_odom_topic_, std::string("/carla/ego_vehicle/odometry"));
        pnh_.param<std::string>("trailers_topic", trailers_topic_, std::string("/trailers/poses"));
        pnh_.param<std::string>("release_topic", release_topic_, std::string("/nmpc/release"));
        pnh_.param<std::string>("control_topic", control_topic_, std::string("/carla/ego_vehicle/vehicle_control_cmd"));
        pnh_.param<std::string>("tractor_speed_topic", tractor_speed_topic_, std::string("/nmpc/tractor_speed"));
        pnh_.param<std::string>("front_steer_topic", front_steer_topic_, std::string("/nmpc/front_steer_angle"));
        pnh_.param<std::string>("trailers_pose_topic", trailers_pose_topic_, std::string("/nmpc/trailers/poses"));
        pnh_.param<std::string>("trailers_yaw_topic", trailers_yaw_topic_, std::string("/nmpc/trailers/yaws"));
        pnh_.param<std::string>("trailers_state_topic", trailers_state_topic_, std::string("/nmpc/trailers/states"));
        pnh_.param<std::string>("decision_time_topic", decision_time_topic_, std::string("/nmpc/decision_time_ms"));
        pnh_.param<std::string>("control_debug_topic", control_debug_topic_, std::string("/nmpc/control_debug"));

        const int raw_count = load_path_from_csv(csv_path_.c_str(), raw_path_, MAX_PATH_POINTS);
        if (raw_count <= 0) {
            throw std::runtime_error("无法加载参考路径: " + csv_path_);
        }

        reference_count_ = prepare_reference_path(raw_path_, raw_count, reference_path_, MAX_REF_POINTS);
        if (reference_count_ <= 0) {
            throw std::runtime_error("无法生成平滑参考路径");
        }

        init_parking_runtime(&runtime_, reference_path_, reference_count_, wheelbase_);

        vehicle_sub_ = nh_.subscribe(vehicle_odom_topic_, 1, &RosTrailerParkingNode::vehicleCallback, this);
        trailers_sub_ = nh_.subscribe(trailers_topic_, 1, &RosTrailerParkingNode::trailersCallback, this);
        release_sub_ = nh_.subscribe(release_topic_, 1, &RosTrailerParkingNode::releaseCallback, this);
        control_pub_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>(control_topic_, 1);
        tractor_speed_pub_ = nh_.advertise<std_msgs::Float64>(tractor_speed_topic_, 1);
        front_steer_pub_ = nh_.advertise<std_msgs::Float64>(front_steer_topic_, 1);
        trailers_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(trailers_pose_topic_, 1);
        trailers_yaw_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(trailers_yaw_topic_, 1);
        trailers_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(trailers_state_topic_, 1);
        decision_time_pub_ = nh_.advertise<std_msgs::Float64>(decision_time_topic_, 1);
        control_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(control_debug_topic_, 1);

        ROS_INFO_STREAM("已加载参考路径点数: " << reference_count_);
        ROS_INFO("参考路径起点: (%.3f, %.3f), 终点/停车点: (%.3f, %.3f)",
                 reference_path_[0].x,
                 reference_path_[0].y,
                 reference_path_[reference_count_ - 1].x,
                 reference_path_[reference_count_ - 1].y);
        ROS_INFO_STREAM("挂车状态是否必需: " << (require_trailers_ ? "是" : "否"));
        ROS_INFO_STREAM("控制输出话题: " << control_topic_);
    }

    void spin() {
        ros::Rate rate(rate_hz_);
        while (ros::ok()) {
            ros::spinOnce();
            step();
            rate.sleep();
        }
    }

private:
    void vehicleCallback(const nav_msgs::OdometryConstPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        vehicle_state_.x = msg->pose.pose.position.x;
        vehicle_state_.y = msg->pose.pose.position.y;

        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;
        const double siny_cosp = 2.0 * (qw * qz + qx * qy);
        const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        vehicle_state_.yaw = std::atan2(siny_cosp, cosy_cosp);
        vehicle_state_.speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        have_vehicle_ = true;
    }

    void trailersCallback(const geometry_msgs::PoseArrayConstPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const int trailer_count = std::min<int>(static_cast<int>(msg->poses.size()), MAX_TRAILERS);
        for (int i = 0; i < MAX_TRAILERS; ++i) {
            trailers_state_[i].alive = 0;
        }

        for (int i = 0; i < trailer_count; ++i) {
            const auto &pose = msg->poses[i];
            trailers_state_[i].x = pose.position.x;
            trailers_state_[i].y = pose.position.y;

            const double qx = pose.orientation.x;
            const double qy = pose.orientation.y;
            const double qz = pose.orientation.z;
            const double qw = pose.orientation.w;
            const double siny_cosp = 2.0 * (qw * qz + qx * qy);
            const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            trailers_state_[i].yaw = std::atan2(siny_cosp, cosy_cosp);
            trailers_state_[i].alive = 1;
        }

        have_trailers_ = true;
    }

    void releaseCallback(const std_msgs::BoolConstPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->data) {
            release_requested_ = true;
        }
    }

    void step() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_vehicle_ || (require_trailers_ && !have_trailers_)) {
            return;
        }

        ParkingStepResult result;
        const auto decision_start = std::chrono::steady_clock::now();
        parking_runtime_step(
            &runtime_,
            &vehicle_state_,
            trailers_state_,
            MAX_TRAILERS,
            release_requested_ ? 1 : 0,
            dt_,
            &result);
        const auto decision_end = std::chrono::steady_clock::now();
        const double decision_time_ms =
            std::chrono::duration<double, std::milli>(decision_end - decision_start).count();

        release_requested_ = false;

        carla_msgs::CarlaEgoVehicleControl control;
        control.throttle = result.command.throttle;
        control.brake = result.command.brake;
        control.steer = result.command.steer;
        control.hand_brake = false;
        control.reverse = false;
        control.manual_gear_shift = false;

        control_pub_.publish(control);

        std_msgs::Float64 tractor_speed_msg;
        tractor_speed_msg.data = vehicle_state_.speed;
        tractor_speed_pub_.publish(tractor_speed_msg);

        std_msgs::Float64 front_steer_msg;
        front_steer_msg.data = result.command.steer * max_steer_rad_;
        front_steer_pub_.publish(front_steer_msg);

        std_msgs::Float64 decision_time_msg;
        decision_time_msg.data = decision_time_ms;
        decision_time_pub_.publish(decision_time_msg);

        std_msgs::Float64MultiArray control_debug_msg;
        control_debug_msg.data.push_back(result.target_speed);
        control_debug_msg.data.push_back(result.command.throttle);
        control_debug_msg.data.push_back(result.command.brake);
        control_debug_msg.data.push_back(result.command.steer);
        control_debug_msg.data.push_back(result.command.steer * max_steer_rad_);
        control_debug_msg.data.push_back(decision_time_ms);
        control_debug_msg.data.push_back(static_cast<double>(result.active_trailer_idx));
        control_debug_msg.data.push_back(static_cast<double>(result.current_target_trailer_idx));
        control_debug_msg.data.push_back(static_cast<double>(result.is_stopped));
        control_debug_msg.data.push_back(runtime_.controller.last_cost);
        control_debug_pub_.publish(control_debug_msg);

        geometry_msgs::PoseArray trailers_pose_msg;
        trailers_pose_msg.header.stamp = ros::Time::now();
        trailers_pose_msg.header.frame_id = "map";

        std_msgs::Float64MultiArray trailers_yaw_msg;
        std_msgs::Float64MultiArray trailers_state_msg;
        for (int i = 0; i < MAX_TRAILERS; ++i) {
            if (!trailers_state_[i].alive) {
                continue;
            }

            geometry_msgs::Pose pose;
            pose.position.x = trailers_state_[i].x;
            pose.position.y = trailers_state_[i].y;
            pose.position.z = 0.0;

            const double half_yaw = trailers_state_[i].yaw * 0.5;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = std::sin(half_yaw);
            pose.orientation.w = std::cos(half_yaw);

            trailers_pose_msg.poses.push_back(pose);
            trailers_yaw_msg.data.push_back(trailers_state_[i].yaw);
            trailers_state_msg.data.push_back(static_cast<double>(i + 1));
            trailers_state_msg.data.push_back(trailers_state_[i].x);
            trailers_state_msg.data.push_back(trailers_state_[i].y);
            trailers_state_msg.data.push_back(trailers_state_[i].yaw);
            trailers_state_msg.data.push_back(trailers_state_[i].yaw * 57.29577951308232);
        }

        trailers_pose_pub_.publish(trailers_pose_msg);
        trailers_yaw_pub_.publish(trailers_yaw_msg);
        trailers_state_pub_.publish(trailers_state_msg);

        ROS_INFO_THROTTLE(
            1.0,
            "目标挂车=%d 当前目标=%d 停车=%d 目标速度=%.2f 距离=%.2f 油门=%.2f 刹车=%.2f 转向=%.2f 决策耗时=%.3fms",
            result.active_trailer_idx,
            result.current_target_trailer_idx,
            result.is_stopped,
            result.target_speed,
            result.distance_to_parking,
            result.command.throttle,
            result.command.brake,
            result.command.steer,
            decision_time_ms);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber vehicle_sub_;
    ros::Subscriber trailers_sub_;
    ros::Subscriber release_sub_;
    ros::Publisher control_pub_;
    ros::Publisher tractor_speed_pub_;
    ros::Publisher front_steer_pub_;
    ros::Publisher trailers_pose_pub_;
    ros::Publisher trailers_yaw_pub_;
    ros::Publisher trailers_state_pub_;
    ros::Publisher decision_time_pub_;
    ros::Publisher control_debug_pub_;

    std::mutex mutex_;
    std::string csv_path_;
    std::string vehicle_odom_topic_;
    std::string trailers_topic_;
    std::string release_topic_;
    std::string control_topic_;
    std::string tractor_speed_topic_;
    std::string front_steer_topic_;
    std::string trailers_pose_topic_;
    std::string trailers_yaw_topic_;
    std::string trailers_state_topic_;
    std::string decision_time_topic_;
    std::string control_debug_topic_;
    double wheelbase_;
    double dt_;
    double rate_hz_;
    double max_steer_rad_ = 0.5235987755982988;
    bool require_trailers_;

    bool have_vehicle_;
    bool have_trailers_;
    bool release_requested_;

    PathPoint raw_path_[MAX_PATH_POINTS];
    PathPoint reference_path_[MAX_REF_POINTS];
    int reference_count_;
    ParkingRuntime runtime_;
    VehicleState vehicle_state_{};
    TrailerState trailers_state_[MAX_TRAILERS]{};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_trailer_parking_node");
    try {
        RosTrailerParkingNode node;
        node.spin();
    } catch (const std::exception &exception) {
        ROS_ERROR_STREAM("启动失败: " << exception.what());
        return 1;
    }

    return 0;
}
