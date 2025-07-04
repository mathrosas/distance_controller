#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "eigen3/Eigen/Dense"
#include "rosidl_runtime_c/message_initialization.h"

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node {
public:
  DistanceController(int scene_number)
      : Node("distance_controller"), scene_number_(scene_number) {
    RCLCPP_INFO(get_logger(), "Distance controller node.");

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odom_callback, this,
                  std::placeholders::_1));
    // sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/rosbot_xl_base_controller/odom", 10,
    //     std::bind(&DistanceController::odom_callback, this,
    //               std::placeholders::_1));

    select_waypoints();
  }

  void run() {
    // PID gains
    const double Kp = 0.5, Ki = 0.05, Kd = 0.1;
    double integral = 0.0, prev_error = 0.0;

    const double I_MAX = 1.0; // integral clamp
    const double V_MAX = 0.4; // m/s

    double goal_x = x_;
    double goal_y = y_;

    geometry_msgs::msg::Twist twist;

    // wait for subscribers
    while (pub_->get_subscription_count() == 0) {
      rclcpp::sleep_for(100ms);
    }
    auto t0 = std::chrono::steady_clock::now();

    for (auto [dx_rel, dy_rel, _] : motions_) {
      goal_x += dx_rel;
      goal_y += dy_rel;

      integral = 0.0;
      prev_error = 0.0;

      // #### main loop: distance > tolerance
      while (true) {
        // compute dt
        auto t1 = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(t1 - t0).count();
        t0 = t1;
        if (dt <= 0)
          dt = 1e-3;

        // radial error
        double ex = goal_x - x_;
        double ey = goal_y - y_;
        double dist = std::hypot(ex, ey);
        if (dist <= pos_tol)
          break;

        // PID on distance
        integral = std::clamp(integral + dist * dt, -I_MAX, I_MAX);
        double derivative = (dist - prev_error) / dt;
        double v = Kp * dist + Ki * integral + Kd * derivative;
        v = std::clamp(v, -V_MAX, V_MAX);
        prev_error = dist;

        // direction unit‐vector
        double ux = ex / dist;
        double uy = ey / dist;

        // body‐frame velocities
        double vx = v * ux;
        double vy = v * uy;

        twist.linear.x = vx;
        twist.linear.y = vy;
        twist.angular.z = 0.0;

        pub_->publish(twist);

        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(25ms);

        RCLCPP_INFO(get_logger(), "dist=%.2f -> v=%.2f (vx=%.2f, vy=%.2f)",
                    dist, v, vx, vy);
      }

      stop();
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::vector<std::tuple<double, double, double>> motions_;
  double x_, y_;
  double w_, l_, r_;

  double pos_tol = 0.01;

  int scene_number_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
  }

  void stop() {
    geometry_msgs::msg::Twist twist;
    rclcpp::Rate rate(20);
    for (int i = 0; i < 20; ++i) {
      pub_->publish(twist);
      rclcpp::spin_some(shared_from_this());
      rate.sleep();
    }
    RCLCPP_INFO(get_logger(), "Stop (zeroed for 0.5 s)");
  }

  void select_waypoints() {
    switch (scene_number_) {
    case 1: // Simulation
      motions_ = {{0.0, 1.0, 0},    {0.0, -1.0, 0.0}, {0.0, -1.0, 0.0},
                  {0.0, 1.0, 0.0},  {1.0, 1.0, 0.0},  {-1.0, -1.0, 0.0},
                  {1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {1.0, 0.0, 0.0},
                  {-1.0, 0.0, 0.0}};
      break;

    case 2: // CyberWorld
      motions_ = {{0.88, 0.0, 0},
                  {0.0, -0.60, 0.0},
                  {0.0, 0.60, 0.0},
                  {-0.88, 0.0, 0.0}};
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int scene_number = 1;
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }
  auto node = std::make_shared<DistanceController>(scene_number);
  node->run();
  rclcpp::shutdown();
  return 0;
}