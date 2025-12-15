#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <algorithm>
#include <cmath>

class LineFollower : public rclcpp::Node
{
public:
  LineFollower()
  : Node("line_follower"),
    start_time_(this->now()),
    last_line_time_(this->now()),
    wait_seconds_(this->declare_parameter<double>("wait_seconds", 5.0)),
    linear_speed_(this->declare_parameter<double>("linear_speed", 8.00)),
    angular_scale_(this->declare_parameter<double>("angular_scale", 0.008)), //0.0380
    angular_turn_rate_(this->declare_parameter<double>("angular_turn_rate", 3.3)),
    coverage_threshold_(this->declare_parameter<double>("coverage_threshold", 0.230)),
    max_error_(this->declare_parameter<double>("max_error", 130.0)),
    min_linear_speed_(this->declare_parameter<double>("min_linear_speed", 0.5)),
    no_line_left_speed_(this->declare_parameter<double>("no_line_left_speed", -0.2)),
    no_line_right_speed_(this->declare_parameter<double>("no_line_right_speed", 1.8)),
    wheel_separation_(this->declare_parameter<double>("wheel_separation", 0.287)),
    no_line_linear_speed_(this->declare_parameter<double>("no_line_linear_speed", 1.0)),
    line_reset_timeout_(this->declare_parameter<double>("line_reset_timeout", 2.0)),
    last_error_(0.0),
    have_error_(false),
    tracked_cx_(0.0),
    is_tracking_(false),
    tracking_confidence_(0)
  {
    detection_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/camera/line_detection",
      10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        this->detectionCallback(msg);
      });
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    this->declare_parameter<double>("max_acceleration", 1.5);

    RCLCPP_INFO(this->get_logger(), "line_follower node ready (waiting %.1fs before control)", wait_seconds_);
  }

private:
  bool ready() const
  {
    return (this->now() - start_time_).seconds() >= wait_seconds_;
  }

  void detectionCallback(const geometry_msgs::msg::Vector3::SharedPtr &msg)
  {
    if (!ready()) {
      return;
    }

    const bool has_line = msg->z > 0.5;
    geometry_msgs::msg::Twist twist;

    if (has_line) {
      const double measured_cx = msg->x;
      const double coverage = msg->y;
      double error = measured_cx;
      last_line_time_ = this->now();

      const bool accepted = shouldAcceptNewLine(measured_cx);
      if (accepted || is_tracking_) {
        error = tracked_cx_;
      }

      last_error_ = error;
      have_error_ = true;

      if (coverage >= coverage_threshold_) {
        twist.linear.x = 1.0;
        twist.angular.z = (error < 0.0) ? angular_turn_rate_ : -angular_turn_rate_;
      } else {
        double speed_factor = 1.0 - std::min(std::abs(error) / max_error_, 0.5);
        double target_speed = linear_speed_ * speed_factor;
        target_speed = std::max(target_speed, min_linear_speed_);

        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "error=%.3f, speed_factor=%.3f, target_speed=%.3f",
          error, speed_factor, target_speed);

        twist.linear.x = target_speed;
        twist.angular.z = -error * angular_scale_;
      }
    } else {
      double vl = no_line_left_speed_;
      double vr = no_line_right_speed_;
      if (have_error_) {
        const bool turn_right = last_error_ < 0.0;
        if (turn_right) {
          vl = no_line_left_speed_;
          vr = no_line_right_speed_;
        } else {
          vl = no_line_right_speed_;
          vr = no_line_left_speed_;
        }
      }

      if (wheel_separation_ > 0.0) {
        twist.angular.z = (vr - vl) / wheel_separation_;
      } else {
        twist.angular.z = 0.0;
      }
      twist.linear.x = no_line_linear_speed_;

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No line detected");
      if (is_tracking_) {
        tracking_confidence_ = std::max(tracking_confidence_ - 1, 0);
        if (tracking_confidence_ <= 0) {
          is_tracking_ = false;
        }
      }
      const double no_line_duration = (this->now() - last_line_time_).seconds();
      if (no_line_duration >= line_reset_timeout_) {
        resetTracking();
      }
    }

    cmd_pub_->publish(twist);
  }

  bool shouldAcceptNewLine(double new_cx)
  {
    if (!is_tracking_) {
      tracked_cx_ = new_cx;
      is_tracking_ = true;
      tracking_confidence_ = MIN_CONFIDENCE;
      return true;
    }

    double distance = std::abs(new_cx - tracked_cx_);

    if (distance < MAX_JUMP) {
      tracked_cx_ = new_cx;
      tracking_confidence_ = std::min(tracking_confidence_ + 1, MAX_CONFIDENCE);
      return true;
    }

    tracking_confidence_--;
    if (tracking_confidence_ <= 0) {
      tracked_cx_ = new_cx;
      is_tracking_ = true;
      tracking_confidence_ = MIN_CONFIDENCE;
      return true;
    }

    return false;
  }

  void resetTracking()
  {
    tracked_cx_ = 0.0;
    is_tracking_ = false;
    tracking_confidence_ = 0;
  }

  rclcpp::Time start_time_;
  rclcpp::Time last_line_time_;
  double wait_seconds_;
  double linear_speed_;
  double angular_scale_;
  double angular_turn_rate_;
  double coverage_threshold_;
  double max_error_;
  double min_linear_speed_;
  double no_line_left_speed_;
  double no_line_right_speed_;
  double wheel_separation_;
  double no_line_linear_speed_;
  double line_reset_timeout_;
  double last_error_;
  bool have_error_;
  double tracked_cx_;
  bool is_tracking_;
  int tracking_confidence_;
  static constexpr int MIN_CONFIDENCE = 15;
  static constexpr int MAX_CONFIDENCE = 18;
  static constexpr double MAX_JUMP = 140.0;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr detection_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
