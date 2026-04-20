#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class WheelMpsMixer : public rclcpp::Node
{
public:
  WheelMpsMixer() : rclcpp::Node("wheel_mps_mixer")
  {
    // --- Parameters ---
    front_track_width_ = this->declare_parameter<double>("front_track_width", 0.75); // m
    rear_track_width_  = this->declare_parameter<double>("rear_track_width",  1.17); // m

    // Optional yaw tuning gains. Keep at 1.0 for geometry-first mixing.
    turn_gain_front_ = this->declare_parameter<double>("turn_gain_front", 1.0);
    turn_gain_rear_  = this->declare_parameter<double>("turn_gain_rear",  1.0);
    in_place_linear_threshold_ = this->declare_parameter<double>("in_place_linear_threshold", 0.05);
    in_place_angular_threshold_ = this->declare_parameter<double>("in_place_angular_threshold", 0.05);
    turn_gain_front_in_place_ = this->declare_parameter<double>("turn_gain_front_in_place", 1.0);
    turn_gain_rear_in_place_  = this->declare_parameter<double>("turn_gain_rear_in_place",  1.0);

    // Per-wheel scale for small calibration trims.
    front_left_scale_  = this->declare_parameter<double>("front_left_scale", 1.0);
    front_right_scale_ = this->declare_parameter<double>("front_right_scale", 1.0);
    rear_left_scale_   = this->declare_parameter<double>("rear_left_scale", 1.0);
    rear_right_scale_  = this->declare_parameter<double>("rear_right_scale", 1.0);

    // Body command limits.
    max_v_ = this->declare_parameter<double>("max_v", 1.0); // m/s
    max_w_ = this->declare_parameter<double>("max_w", 0.8); // rad/s

    // Per-wheel limits aligned with the tuned firmware defaults.
    max_wheel_fl_mps_ = this->declare_parameter<double>("max_wheel_fl_mps", 1.36);
    max_wheel_fr_mps_ = this->declare_parameter<double>("max_wheel_fr_mps", 1.33);
    max_wheel_rl_mps_ = this->declare_parameter<double>("max_wheel_rl_mps", 6.30);
    max_wheel_rr_mps_ = this->declare_parameter<double>("max_wheel_rr_mps", 6.85);

    // Ramp limit.
    max_accel_mps2_ = this->declare_parameter<double>("max_accel_mps2", 1.0); // m/s^2 per wheel

    // Watchdog.
    cmd_timeout_ms_ = this->declare_parameter<int>("cmd_timeout_ms", 300);

    // Topics.
    input_topic_  = this->declare_parameter<std::string>("input_topic",  "/cmd_vel");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/wheel_cmd");

    // Direction per wheel (+1 normal, -1 invert if motor mounted reversed).
    dir_fl_ = this->declare_parameter<int>("dir_fl", 1);
    dir_fr_ = this->declare_parameter<int>("dir_fr", 1);
    dir_rl_ = this->declare_parameter<int>("dir_rl", 1);
    dir_rr_ = this->declare_parameter<int>("dir_rr", 1);

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, 10);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, 10,
      std::bind(&WheelMpsMixer::on_cmd, this, std::placeholders::_1));

    target_mps_ = {0.0, 0.0, 0.0, 0.0};
    out_mps_    = {0.0, 0.0, 0.0, 0.0};
    last_cmd_time_    = this->now();
    last_update_time_ = this->now();

    double hz = std::max(1.0, publish_rate_hz_);
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));

    timer_ = this->create_wall_timer(period_ns, std::bind(&WheelMpsMixer::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "wheel_mps_mixer in=%s out=%s front_track=%.3f rear_track=%.3f max_v=%.2f max_w=%.2f in_place_lin<=%.2f in_place_ang>=%.2f",
      input_topic_.c_str(), output_topic_.c_str(), front_track_width_, rear_track_width_, max_v_, max_w_,
      in_place_linear_threshold_, in_place_angular_threshold_);
  }

private:
  static double clamp(double x, double lo, double hi)
  {
    return std::min(std::max(x, lo), hi);
  }

  double compute_common_scale(const std::array<double, 4> & wheel_mps) const
  {
    const std::array<double, 4> limits = {
      max_wheel_fl_mps_,
      max_wheel_fr_mps_,
      max_wheel_rl_mps_,
      max_wheel_rr_mps_
    };

    double scale = 1.0;
    for (std::size_t i = 0; i < wheel_mps.size(); ++i) {
      const double mag = std::abs(wheel_mps[i]);
      if (mag <= 1e-9) {
        continue;
      }

      const double limit = limits[i];
      if (limit <= 0.0) {
        return 0.0;
      }

      if (mag > limit) {
        scale = std::min(scale, limit / mag);
      }
    }

    return scale;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double v = clamp(msg->linear.x,  -max_v_, max_v_);
    const double w = clamp(msg->angular.z, -max_w_, max_w_);

    const bool in_place_turn =
      std::abs(v) <= in_place_linear_threshold_ &&
      std::abs(w) >= in_place_angular_threshold_;

    const double front_turn_gain = in_place_turn ? turn_gain_front_in_place_ : turn_gain_front_;
    const double rear_turn_gain  = in_place_turn ? turn_gain_rear_in_place_  : turn_gain_rear_;

    const double wf = w * front_turn_gain;
    const double wr = w * rear_turn_gain;

    std::array<double, 4> wheel_mps = {
      (v - wf * (front_track_width_ * 0.5)) * front_left_scale_,
      (v + wf * (front_track_width_ * 0.5)) * front_right_scale_,
      (v - wr * (rear_track_width_  * 0.5)) * rear_left_scale_,
      (v + wr * (rear_track_width_  * 0.5)) * rear_right_scale_
    };

    // Scale all 4 wheels together so the requested curvature is preserved.
    const double scale = compute_common_scale(wheel_mps);
    for (double & wheel : wheel_mps) {
      wheel *= scale;
    }

    target_mps_[0] = wheel_mps[0] * static_cast<double>(dir_fl_);
    target_mps_[1] = wheel_mps[1] * static_cast<double>(dir_fr_);
    target_mps_[2] = wheel_mps[2] * static_cast<double>(dir_rl_);
    target_mps_[3] = wheel_mps[3] * static_cast<double>(dir_rr_);

    last_cmd_time_ = this->now();
  }

  void on_timer()
  {
    const rclcpp::Time now = this->now();
    const double dt = (now - last_update_time_).seconds();
    if (dt <= 0.0) {
      last_update_time_ = now;
      return;
    }
    last_update_time_ = now;

    double age_ms = (now - last_cmd_time_).seconds() * 1000.0;
    std::array<double, 4> desired = target_mps_;
    if (age_ms > static_cast<double>(cmd_timeout_ms_)) {
      desired = {0.0, 0.0, 0.0, 0.0};
    }

    const double dv_max = max_accel_mps2_ * dt;
    for (int i = 0; i < 4; ++i) {
      const double dv = desired[i] - out_mps_[i];
      const double step = clamp(dv, -dv_max, dv_max);
      out_mps_[i] += step;
    }

    std_msgs::msg::Float32MultiArray out;
    out.data = {
      static_cast<float>(out_mps_[0]),
      static_cast<float>(out_mps_[1]),
      static_cast<float>(out_mps_[2]),
      static_cast<float>(out_mps_[3])
    };
    pub_->publish(out);
  }

  // Params.
  double front_track_width_, rear_track_width_;
  double turn_gain_front_, turn_gain_rear_;
  double in_place_linear_threshold_, in_place_angular_threshold_;
  double turn_gain_front_in_place_, turn_gain_rear_in_place_;
  double front_left_scale_, front_right_scale_, rear_left_scale_, rear_right_scale_;
  double max_v_, max_w_;
  double max_wheel_fl_mps_, max_wheel_fr_mps_, max_wheel_rl_mps_, max_wheel_rr_mps_;
  double max_accel_mps2_;
  int cmd_timeout_ms_;
  double publish_rate_hz_;
  int dir_fl_, dir_fr_, dir_rl_, dir_rr_;
  std::string input_topic_, output_topic_;

  // State.
  std::array<double, 4> target_mps_;
  std::array<double, 4> out_mps_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_update_time_;

  // ROS.
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelMpsMixer>());
  rclcpp::shutdown();
  return 0;
}
