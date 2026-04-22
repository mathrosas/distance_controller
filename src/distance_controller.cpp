#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <vector>

using namespace std::chrono_literals;

static inline float wrapPi(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

class DistanceController : public rclcpp::Node {
public:
  DistanceController(int scene_number)
      : Node("distance_controller"), scene_number_(scene_number),
        got_odom_(false), wp_reached_(false), init_(true), paused_(false),
        has_last_time_(false), target_wp_(0) {
    twist_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odomCallback, this,
                  std::placeholders::_1));
    timer_ = this->create_wall_timer(
        50ms, std::bind(&DistanceController::executeCallback, this));

    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    SelectWaypoints();
  }

  void stopRobot() {
    auto stop = geometry_msgs::msg::Twist();
    for (int i = 0; i < 5; ++i) {
      twist_pub_->publish(stop);
    }
  }

private:
  void SelectWaypoints() {
    // Waypoints [dx, dy, dphi] expressed in BODY frame
    switch (scene_number_) {
    case 1: // Simulation
      waypoints_ = {
          {0.0f, 1.0f, 0.0f},   // w1
          {0.0f, -1.0f, 0.0f},  // w2
          {0.0f, -1.0f, 0.0f},  // w3
          {0.0f, 1.0f, 0.0f},   // w4
          {1.0f, 1.0f, 0.0f},   // w5
          {-1.0f, -1.0f, 0.0f}, // w6
          {1.0f, -1.0f, 0.0f},  // w7
          {-1.0f, 1.0f, 0.0f},  // w8
          {1.0f, 0.0f, 0.0f},   // w9
          {-1.0f, 0.0f, 0.0f}   // w10
      };
      break;

    case 2: // CyberWorld
      waypoints_ = {
          {0.90f, 0.0f, 0.0f},  // w1
          {0.0f, -0.55f, 0.0f}, // w2
          {0.0f, 0.55f, 0.0f},  // w3
          {-0.90f, 0.0f, 0.0f}  // w4
      };
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto &o = msg->pose.pose.orientation;
    tf2::Quaternion q(o.x, o.y, o.z, o.w);
    current_pose_(0) = msg->pose.pose.position.x;
    current_pose_(1) = msg->pose.pose.position.y;
    current_pose_(2) = tf2::impl::getYaw(q);
    got_odom_ = true;
  }

  void setTargetFromCurrent() {
    // Body-frame (dx,dy) rotated into world frame, then added to current pose.
    const float c = std::cos(current_pose_(2));
    const float s = std::sin(current_pose_(2));
    const Eigen::Vector3f &w = waypoints_[target_wp_];
    target_pose_(0) = current_pose_(0) + c * w(0) - s * w(1);
    target_pose_(1) = current_pose_(1) + s * w(0) + c * w(1);
    target_pose_(2) = wrapPi(current_pose_(2) + w(2));
  }

  void resetPIDState() {
    prev_error_.setZero();
    integral_error_.setZero();
    d_filt_.setZero();
    has_last_time_ = false;
  }

  void executeCallback() {
    if (!got_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000,
                           "Odom data not received!");
      return;
    }

    const rclcpp::Time now = clock_->now();

    // Paused state between waypoints
    if (paused_) {
      if ((now - pause_time_).seconds() >= 2.0) {
        paused_ = false;
        resetPIDState();
      } else {
        auto stop = geometry_msgs::msg::Twist();
        twist_pub_->publish(stop);
        return;
      }
    }

    // Latch target waypoint on first cycle and when previous was reached.
    if (wp_reached_ || init_) {
      setTargetFromCurrent();
      init_ = false;
      wp_reached_ = false;
      resetPIDState();
      RCLCPP_INFO(this->get_logger(),
                  "WP %zu target (world): (%.3f, %.3f, %.3f)", target_wp_ + 1,
                  target_pose_(0), target_pose_(1), target_pose_(2));
    }

    // Time step
    double dt = 0.05;
    if (has_last_time_) {
      dt = (now - last_time_).seconds();
    }
    dt = std::clamp(dt, 1e-3, 0.2);
    last_time_ = now;
    has_last_time_ = true;
    const float dtf = static_cast<float>(dt);

    // World-frame translational error
    Eigen::Vector2f err_world{target_pose_(0) - current_pose_(0),
                              target_pose_(1) - current_pose_(1)};

    // Rotate world error into BODY frame so the PID lives in the same frame
    // as cmd_vel (body). This keeps saturation bounds physical.
    const float c = std::cos(current_pose_(2));
    const float s = std::sin(current_pose_(2));
    Eigen::Vector2f err_body{c * err_world(0) + s * err_world(1),
                             -s * err_world(0) + c * err_world(1)};

    // Acceptance: position only. This controller does not command yaw, so
    // gating on yaw would permanently block acceptance if odom yaw drifts.
    const float dpos = err_body.norm();
    if (dpos < pos_tol_) {
      auto stop = geometry_msgs::msg::Twist();
      twist_pub_->publish(stop);
      wp_reached_ = true;
      target_wp_++;

      if (target_wp_ >= waypoints_.size()) {
        RCLCPP_INFO(this->get_logger(), "Final waypoint reached!");
        stopRobot();
        rclcpp::shutdown();
        return;
      }
      pause_time_ = now;
      paused_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Waypoint %zu reached (dpos=%.3f m), pausing...", target_wp_,
                  dpos);
      return;
    }

    // Filtered derivative (body frame)
    const Eigen::Vector2f prev_body{prev_error_(0), prev_error_(1)};
    const Eigen::Vector2f d_raw = (err_body - prev_body) / dtf;
    const Eigen::Vector2f d_f{
        alpha_d_ * d_raw(0) + (1.0f - alpha_d_) * d_filt_(0),
        alpha_d_ * d_raw(1) + (1.0f - alpha_d_) * d_filt_(1)};
    d_filt_(0) = d_f(0);
    d_filt_(1) = d_f(1);

    // PID (body frame)
    const Eigen::Vector2f I{integral_error_(0), integral_error_(1)};
    Eigen::Vector2f V_unsat = Kp_ * err_body + Kd_ * d_f + Ki_ * I;

    // Saturate in body frame (per-axis, matches wheel/actuator limits)
    Eigen::Vector2f V_sat{std::clamp(V_unsat(0), -max_lin_vel_, max_lin_vel_),
                          std::clamp(V_unsat(1), -max_lin_vel_, max_lin_vel_)};

    // Conditional-integration anti-windup (body frame)
    auto sameSign = [](float a, float b) {
      return (a > 0 && b > 0) || (a < 0 && b < 0);
    };
    const bool x_sat = (std::abs(V_unsat(0)) > std::abs(V_sat(0)) + 1e-4f) &&
                       sameSign(V_unsat(0), err_body(0));
    const bool y_sat = (std::abs(V_unsat(1)) > std::abs(V_sat(1)) + 1e-4f) &&
                       sameSign(V_unsat(1), err_body(1));
    if (!x_sat)
      integral_error_(0) += err_body(0) * dtf;
    if (!y_sat)
      integral_error_(1) += err_body(1) * dtf;
    integral_error_(0) =
        std::clamp(integral_error_(0), -int_limit_, int_limit_);
    integral_error_(1) =
        std::clamp(integral_error_(1), -int_limit_, int_limit_);
    integral_error_(2) = 0.0f;

    prev_error_(0) = err_body(0);
    prev_error_(1) = err_body(1);
    prev_error_(2) = 0.0f;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = V_sat(0);
    cmd_vel.linear.y = V_sat(1);
    cmd_vel.angular.z = 0.0;
    twist_pub_->publish(cmd_vel);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time pause_time_;
  rclcpp::Time last_time_;
  int scene_number_;
  bool got_odom_, wp_reached_, init_, paused_, has_last_time_;
  size_t target_wp_;
  std::shared_ptr<rclcpp::Clock> clock_;
  Eigen::Vector3f current_pose_{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f target_pose_{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f prev_error_{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f integral_error_{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f d_filt_{0.0f, 0.0f, 0.0f};
  std::vector<Eigen::Vector3f> waypoints_;

  // PID gains and limits
  const float Kp_ = 1.0f;
  const float Ki_ = 0.05f;
  const float Kd_ = 0.08f;
  const float int_limit_ = 0.6f;
  const float alpha_d_ = 0.2f;
  const float max_lin_vel_ = 0.28f;

  // Acceptance tolerance (position-only; yaw is not commanded)
  const float pos_tol_ = 0.03f;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int scene_number = 2; // default: real
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  auto node = std::make_shared<DistanceController>(scene_number);
  rclcpp::spin(node);

  node->stopRobot();
  rclcpp::shutdown();
  return 0;
}
