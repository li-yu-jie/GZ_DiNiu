// Ackermann 里程计节点：
// 订阅驱动轮线速度与前轮转向角，按自行车模型积分车辆位姿，
// 发布标准 ROS2 /odom（nav_msgs/msg/Odometry）。
#include <cmath>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class AckermannOdomNode : public rclcpp::Node {
public:
  AckermannOdomNode() : Node("ackermann_odom_node") {
    // 输入/输出话题与坐标系参数。
    speed_topic_ = declare_parameter<std::string>("speed_topic", "linear_velocity");
    steer_topic_ = declare_parameter<std::string>("steer_topic", "steer_angle");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel_odom");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    // wheelbase_m: 前后轴中心距，用于由转向角换算角速度。
    wheelbase_m_ = declare_parameter<double>("wheelbase_m", 1.38);
    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);
    // 低速保护：速度过小时不再根据 tan(delta) 计算角速度，避免数值抖动。
    min_speed_for_yaw_m_s_ = declare_parameter<double>("min_speed_for_yaw_m_s", 1e-4);

    if (wheelbase_m_ <= 0.0) {
      throw std::runtime_error("wheelbase_m must be > 0");
    }
    if (publish_hz_ <= 0.0) {
      throw std::runtime_error("publish_hz must be > 0");
    }

    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      speed_topic_, 10, std::bind(&AckermannOdomNode::on_speed, this, std::placeholders::_1));
    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
      steer_topic_, 10, std::bind(&AckermannOdomNode::on_steer, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // 定时积分里程计，不依赖输入消息到达频率严格同步。
    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AckermannOdomNode::publish_odom, this));

    last_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "ackermann_odom started. speed_topic=%s steer_topic=%s odom_topic=%s wheelbase=%.3f",
      speed_topic_.c_str(), steer_topic_.c_str(), odom_topic_.c_str(), wheelbase_m_);
  }

private:
  static double normalize_angle(double angle) {
    // 将 yaw 约束到 [-pi, pi]，避免角度无限累积。
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    // 里程计只估计平面运动，因此只保留绕 z 轴的偏航角。
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  void on_speed(const std_msgs::msg::Float64::SharedPtr msg) {
    // 驱动轮速度节点输出的车体前向线速度，单位 m/s。
    linear_velocity_m_s_ = msg->data;
  }

  void on_steer(const std_msgs::msg::Float64::SharedPtr msg) {
    // 转向闭环节点输出的前轮转角，单位 rad。
    steer_angle_rad_ = msg->data;
  }

  void publish_odom() {
    // 固定周期积分：
    // yaw_rate = v * tan(delta) / L
    // x_dot = v * cos(yaw)
    // y_dot = v * sin(yaw)
    //
    // 这里采用单轨/自行车模型近似车辆运动学：
    // - v 为车体前向速度
    // - delta 为前轮等效转角
    // - L 为轴距
    const auto current_time = now();
    const double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_time_ = current_time;

    const double v = linear_velocity_m_s_;
    const double steer = steer_angle_rad_;

    double yaw_rate = 0.0;
    if (std::abs(v) >= min_speed_for_yaw_m_s_) {
      yaw_rate = v * std::tan(steer) / wheelbase_m_;
    }

    // 使用中点法积分，较欧拉法在转弯时更稳一些。
    const double mid_yaw = yaw_ + 0.5 * yaw_rate * dt;
    x_ += v * std::cos(mid_yaw) * dt;
    y_ += v * std::sin(mid_yaw) * dt;
    yaw_ = normalize_angle(yaw_ + yaw_rate * dt);

    // 填充标准 /odom 消息。
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = yaw_to_quaternion(yaw_);

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = yaw_rate;

    odom_pub_->publish(odom_msg);
  }

  // 话题与坐标系配置
  std::string speed_topic_;
  std::string steer_topic_;
  std::string odom_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // 模型与发布参数
  double wheelbase_m_{};
  double publish_hz_{};
  double min_speed_for_yaw_m_s_{};

  // 最近一次输入
  double linear_velocity_m_s_{0.0};
  double steer_angle_rad_{0.0};

  // 累积积分得到的位姿状态
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  // 上一轮积分时间
  rclcpp::Time last_time_;

  // ROS 通信对象
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AckermannOdomNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ackermann_odom startup error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
