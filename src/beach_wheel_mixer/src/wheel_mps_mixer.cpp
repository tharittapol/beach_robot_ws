#include <algorithm>
#include <array>
#include <chrono>
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

    // turning gains (tune on sand)
    turn_gain_front_ = this->declare_parameter<double>("turn_gain_front", 0.7);
    turn_gain_rear_  = this->declare_parameter<double>("turn_gain_rear",  1.0);
    // Per-wheel scale
    front_left_scale_ = this->declare_parameter<double>("front_left_scale", 1.0);
    front_right_scale_ = this->declare_parameter<double>("front_right_scale", 1.0);
    rear_left_scale_ = this->declare_parameter<double>("rear_left_scale", 1.0);
    rear_right_scale_ = this->declare_parameter<double>("rear_right_scale", 1.0);

    // limits
    max_v_         = this->declare_parameter<double>("max_v", 1.0);             // m/s
    max_w_         = this->declare_parameter<double>("max_w", 1.0);             // rad/s
    max_wheel_mps_ = this->declare_parameter<double>("max_wheel_mps", 1.5);     // m/s per wheel

    // ramp limit
    max_accel_mps2_ = this->declare_parameter<double>("max_accel_mps2", 0.8);   // m/s^2 per wheel

    // watchdog
    cmd_timeout_ms_ = this->declare_parameter<int>("cmd_timeout_ms", 300);

    // topics
    input_topic_  = this->declare_parameter<std::string>("input_topic",  "/cmd_vel");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/wheel_cmd");

    // direction per wheel (+1 normal, -1 invert if motor mounted reversed)
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

    // wall timer
    double hz = std::max(1.0, publish_rate_hz_);
    auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));

    timer_ = this->create_wall_timer(period_ns, std::bind(&WheelMpsMixer::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "wheel_mps_mixer in=%s out=%s",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  static double clamp(double x, double lo, double hi)
  {
    return std::min(std::max(x, lo), hi);
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // base command
    double v = clamp(msg->linear.x,  -max_v_, max_v_);
    double w = clamp(msg->angular.z, -max_w_, max_w_);

    double wf = w * turn_gain_front_;
    double wr = w * turn_gain_rear_;

    // wheel linear speeds (m/s)
    double v_fl = v - wf * (front_track_width_ * 0.5);
    double v_fr = v + wf * (front_track_width_ * 0.5);
    double v_rl = v - wr * (rear_track_width_  * 0.5);
    double v_rr = v + wr * (rear_track_width_  * 0.5);
    // per-wheel scale
    v_fl *= front_left_scale_;
    v_fr *= front_right_scale_;
    v_rl *= rear_left_scale_;
    v_rr *= rear_right_scale_;

    // clamp to max wheel speed
    v_fl = clamp(v_fl, -max_wheel_mps_, max_wheel_mps_);
    v_fr = clamp(v_fr, -max_wheel_mps_, max_wheel_mps_);
    v_rl = clamp(v_rl, -max_wheel_mps_, max_wheel_mps_);
    v_rr = clamp(v_rr, -max_wheel_mps_, max_wheel_mps_);

    target_mps_[0] = v_fl * static_cast<double>(dir_fl_);
    target_mps_[1] = v_fr * static_cast<double>(dir_fr_);
    target_mps_[2] = v_rl * static_cast<double>(dir_rl_);
    target_mps_[3] = v_rr * static_cast<double>(dir_rr_);

    last_cmd_time_ = this->now();
  }

  void on_timer()
  {
    rclcpp::Time t = this->now();
    double dt = (t - last_update_time_).seconds();
    if (dt <= 0.0) {
      last_update_time_ = t;
      return;
    }
    last_update_time_ = t;

    // watchdog
    double age_ms = (t - last_cmd_time_).seconds() * 1000.0;
    std::array<double,4> desired = target_mps_;
    if (age_ms > static_cast<double>(cmd_timeout_ms_)) {
      desired = {0.0, 0.0, 0.0, 0.0};
    }

    // ramp limit
    double dv_max = max_accel_mps2_ * dt;
    for (int i = 0; i < 4; i++) {
      double dv = desired[i] - out_mps_[i];
      double step = clamp(dv, -dv_max, dv_max);
      out_mps_[i] += step;
    }

    // publish Float32MultiArray: [FL, FR, RL, RR]
    std_msgs::msg::Float32MultiArray out;
    out.data = {
      static_cast<float>(out_mps_[0]),
      static_cast<float>(out_mps_[1]),
      static_cast<float>(out_mps_[2]),
      static_cast<float>(out_mps_[3])
    };
    pub_->publish(out);
  }

  // params
  double front_track_width_, rear_track_width_;
  double turn_gain_front_, turn_gain_rear_;
  double front_left_scale_, front_right_scale_, rear_left_scale_, rear_right_scale_;
  double max_v_, max_w_, max_wheel_mps_;
  double max_accel_mps2_;
  int cmd_timeout_ms_;
  double publish_rate_hz_;
  int dir_fl_, dir_fr_, dir_rl_, dir_rr_;
  std::string input_topic_, output_topic_;

  // state
  std::array<double,4> target_mps_;
  std::array<double,4> out_mps_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_update_time_;

  // ros
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
