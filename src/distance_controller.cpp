#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <thread>
#include <vector>

struct Waypoint {
  double dx;   // Delta x in meters
  double dy;   // Delta y in meters
  double dyaw; // Delta yaw in radians
};

class DistanceController : public rclcpp::Node {
public:
  DistanceController(int scene_number) : Node("distance_controller") {

    // Setup Waypoints and PID gains depending on scene
    setup_scene(scene_number);

    RCLCPP_INFO(this->get_logger(),
                "PID gains: Kp=(%.3f, %.3f), Ki=(%.3f, %.3f), Kd=(%.3f, %.3f)",
                Kp_x_, Kp_y_, Ki_x_, Ki_y_, Kd_x_, Kd_y_);

    // Create Reentrant Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;

    // Subscriber for Odometry Information
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odom_callback, this,
                  std::placeholders::_1),
        options);

    // Publisher for the CMD Vel
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Distance Controller Node Ready!");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // State
  double current_x_{0.0}, current_y_{0.0};
  double current_vx_{0.0}, current_vy_{0.0};
  double current_yaw_{0.0};
  double segment_target_x_{0.0}, segment_target_y_{0.0};
  bool segment_active_{false};
  bool odom_received_{false};
  std::vector<Waypoint> waypoints_;
  size_t current_idx_{0};

  geometry_msgs::msg::Twist twist_;

  // PID
  double Kp_x_{2.5}, Kp_y_{2.5};
  double Ki_x_{0.005}, Ki_y_{0.005};
  double Kd_x_{0.3}, Kd_y_{0.3};
  double sum_I_x{0.0}, sum_I_y{0.0};

  // Timing & limits
  const double rate_hz_{40.0}; // ROSbot XL and Sim work on 20 Hz
  const double dt_{1.0 / rate_hz_};
  double max_speed_{0.8}; // Took from ROSBot XL Spec
  const double linear_tolerance_{0.01};
  const double speed_tolerance_{0.01};

  const double pause_duration_sec_{1.0};
  const int pause_ticks_goal_{
      static_cast<int>(pause_duration_sec_ * rate_hz_)}; // sec * rateHZ
  int pause_ticks_{0};
  bool pausing_{false};

  void setup_scene(const int scene_number) {
    switch (scene_number) {
    case 1: // Simulation
      waypoints_ = {{0.0, 1.0, 0.0},  {0.0, -1.0, 0.0}, {0.0, -1.0, 0.0},
                    {0.0, 1.0, 0.0},  {1.0, 1.0, 0.0},  {-1.0, -1.0, 0.0},
                    {1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
                    {-1.0, 0.0, 0.0}};
      Kp_x_ = 2.5, Kp_y_ = 2.5;
      Ki_x_ = 0.005, Ki_y_ = 0.005;
      Kd_x_ = 0.3, Kd_y_ = 0.3;
      RCLCPP_INFO(get_logger(),
                  "Scene 1 (Sim): set sim waypoints and PID gains");
      RCLCPP_INFO(this->get_logger(), "Set Simulation Waypoints.");
      break;
    case 2: // CyberWorld
      max_speed_ = 0.45;
      Kp_x_ = 2.0, Kp_y_ = 2.0;
      Ki_x_ = 0.001, Ki_y_ = 0.001;
      Kd_x_ = 0.3, Kd_y_ = 0.3;
      waypoints_ = {{1.0, -0.0, 0.0},
                    {0.0, -0.55, 0.0},
                    {0.0, 0.55, 0.0},
                    {-1.0, +0.0, 0.0}};
      RCLCPP_INFO(get_logger(),
                  "Scene 2 (CyberWorld): set real waypoints and PID gains");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number);
      rclcpp::shutdown();
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_vx_ = msg->twist.twist.linear.x;
    current_vy_ = msg->twist.twist.linear.y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);

    if (!odom_received_) {
      odom_received_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "First odom: x=%.3f y=%.3f. Starting control loop.",
                  current_x_, current_y_);

      // Control Loop Timer 40 Hz, normal for the PID
      control_timer_ = this->create_wall_timer(
          std::chrono::duration<double>(dt_),
          std::bind(&DistanceController::control_loop, this), callback_group_);
    }
  }

  void control_loop() {
    if (!interfaces_ready()) {
      stop();
      return;
    }

    if (pausing_) {
      stop();
      pause_ticks_++;
      if (pause_ticks_ >= pause_ticks_goal_) {
        pausing_ = false;
      }
      return;
    }

    // 1. If no active segment, start one (if waypoints left)
    if (!segment_active_ && current_idx_ < waypoints_.size()) {
      auto wp = waypoints_[current_idx_];

      // Translation in robot frame
      double dx_world =
          wp.dx * std::cos(current_yaw_) - wp.dy * std::sin(current_yaw_);
      double dy_world =
          wp.dx * std::sin(current_yaw_) + wp.dy * std::cos(current_yaw_);

      segment_target_x_ = current_x_ + dx_world;
      segment_target_y_ = current_y_ + dy_world;
      segment_active_ = true;
      sum_I_x = 0.0;
      sum_I_y = 0.0;

      RCLCPP_INFO(this->get_logger(),
                  "Moving to waypoint %zu (dx=%.3f, dy=%.3f)", current_idx_,
                  wp.dx, wp.dy);
    }

    // 2. If no more segments, stop the robot
    if (current_idx_ >= waypoints_.size()) {
      stop();
      RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
      rclcpp::shutdown();
      return;
    }

    // 3. Compute error to current waypoint
    double ex = segment_target_x_ - current_x_;
    double ey = segment_target_y_ - current_y_;
    // rotate in the base frame
    double ex_b = std::cos(current_yaw_) * ex + std::sin(current_yaw_) * ey;
    double ey_b = -std::sin(current_yaw_) * ex + std::cos(current_yaw_) * ey;
    double dex = 0.0 - current_vx_;
    double dey = 0.0 - current_vy_;

    // 4. Check if we reached the waypoint
    if (std::hypot(ex_b, ey_b) < linear_tolerance_ &&
        std::hypot(dex, dey) < speed_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu.", current_idx_);
      current_idx_++;
      segment_active_ = false;
      stop();
      // start pause
      pausing_ = true;
      pause_ticks_ = 0;
      return;
    }

    // 5. Run our PID controller using ex, ey, dex, dey
    compute_PID(ex_b, ey_b, dex, dey);

    // RCLCPP_INFO(this->get_logger(), //*this->get_clock(), 100,
    //             "Debug: Error (ex=%.3f, ey=%.3f) | "
    //             "World Frame Speed: (v_x=%.3f, v_y=%.3f)",
    //             ex, ey, current_vx_, current_vy_);

    cmd_vel_publisher_->publish(twist_);
  }

  bool interfaces_ready() {
    // Is anyone listening to /cmd_vel ?
    auto cmd_vel_subs = cmd_vel_publisher_->get_subscription_count();

    if (cmd_vel_subs == 0) {
      sum_I_x = 0.0;
      sum_I_y = 0.0;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "No subscribers on /cmd_vel. Holding control loop...");
      return false;
    }

    return true;
  }

  void compute_PID(double ex, double ey, double dex, double dey) {
    // Update integral value of integral control
    sum_I_x += ex * dt_;
    sum_I_y += ey * dt_;

    twist_.linear.x = Kp_x_ * ex + Kd_x_ * dex + Ki_x_ * sum_I_x;
    twist_.linear.y = Kp_y_ * ey + Kd_y_ * dey + Ki_y_ * sum_I_y;

    // Limit total linear speed to max_speed_
    double vnorm = std::hypot(twist_.linear.x, twist_.linear.y);
    if (vnorm > max_speed_) {
      twist_.linear.x *= max_speed_ / vnorm;
      twist_.linear.y *= max_speed_ / vnorm;
    }
  }

  void stop() {
    twist_.linear.x = 0.0;
    twist_.linear.y = 0.0;
    cmd_vel_publisher_->publish(twist_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto node = std::make_shared<DistanceController>(scene_number);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);
  executor.add_node(node);
  try {
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}