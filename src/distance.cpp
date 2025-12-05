#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "turtlesim/msg/pose.hpp"

// Struct for keeping turtle-related info
struct TState
{
  turtlesim::msg::Pose pose;
  bool seen_pose = false;

  geometry_msgs::msg::Twist last_cmd;
  bool cmd_received = false;

  // Border logic
  bool went_outside = false;
  bool doing_turn = false;
  int turn_ticks = 0;
  bool wait_for_new_input = false;
};

class DistanceNode : public rclcpp::Node
{
public:
  DistanceNode()
  : Node("distance_node")
  {
    dist_limit_ = this->declare_parameter("dist_limit", 0.75);
    border_min_ = this->declare_parameter("border_min", 1.0);
    border_max_ = this->declare_parameter("border_max", 10.0);
    margin_     = this->declare_parameter("margin", 0.5);

    // Pose subscribers
    sub_t1_pose_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      [this](const turtlesim::msg::Pose::SharedPtr msg) {
        t1_.pose = *msg;
        t1_.seen_pose = true;
      });

    sub_t2_pose_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10,
      [this](const turtlesim::msg::Pose::SharedPtr msg) {
        t2_.pose = *msg;
        t2_.seen_pose = true;
      });

    // UI commands
    sub_t1_input_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel_input", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        t1_.last_cmd = *msg;
        t1_.cmd_received = true;

        if (t1_.wait_for_new_input && !t1_.doing_turn) {
          t1_.wait_for_new_input = false;
        }
      });

    sub_t2_input_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle2/cmd_vel_input", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        t2_.last_cmd = *msg;
        t2_.cmd_received = true;

        if (t2_.wait_for_new_input && !t2_.doing_turn) {
          t2_.wait_for_new_input = false;
        }
      });

    // Publishers
    pub_t1_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pub_t2_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
    pub_dist_   = this->create_publisher<std_msgs::msg::Float32>("/turtles_distance", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&DistanceNode::stepLoop, this));

    RCLCPP_INFO(this->get_logger(), "Distance node ready.");
  }

private:
  // Turtle states
  TState t1_, t2_;

  // Parameters
  double dist_limit_;
  double border_min_;
  double border_max_;
  double margin_;

  // Optional collision push-away
  bool separating_ = false;
  int sep_ticks_   = 0;

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t1_pose_, sub_t2_pose_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_t1_input_, sub_t2_input_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t1_cmd_, pub_t2_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dist_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Main loop
  void stepLoop()
  {
    geometry_msgs::msg::Twist out1, out2;

    // Handle separation first
    if (separating_) {
      runSeparation(out1, out2);
      pub_t1_cmd_->publish(out1);
      pub_t2_cmd_->publish(out2);
      return;
    }

    // Begin from UI
    if (t1_.cmd_received) out1 = t1_.last_cmd;
    if (t2_.cmd_received) out2 = t2_.last_cmd;

    // Distance check
    bool have_both = t1_.seen_pose && t2_.seen_pose;
    double dist = 999.0;
    if (have_both) {
      double dx = t1_.pose.x - t2_.pose.x;
      double dy = t1_.pose.y - t2_.pose.y;
      dist = std::sqrt(dx * dx + dy * dy);

      std_msgs::msg::Float32 msg;
      msg.data = dist;
      pub_dist_->publish(msg);
    }

    if (have_both && dist < dist_limit_) {
      separating_ = true;
      sep_ticks_ = 30;
      out1 = geometry_msgs::msg::Twist();
      out2 = geometry_msgs::msg::Twist();
    } else {
      borderLogic(t1_, out1, "turtle1");
      borderLogic(t2_, out2, "turtle2");
    }

    pub_t1_cmd_->publish(out1);
    pub_t2_cmd_->publish(out2);
  }

  // Border logic: stop → turn 180° → wait
  void borderLogic(TState & t, geometry_msgs::msg::Twist & cmd, const std::string & name)
  {
    if (!t.seen_pose) return;

    bool outside =
      t.pose.x <= border_min_ || t.pose.x >= border_max_ ||
      t.pose.y <= border_min_ || t.pose.y >= border_max_;

    bool comfortably_inside =
      t.pose.x > (border_min_ + margin_) &&
      t.pose.x < (border_max_ - margin_) &&
      t.pose.y > (border_min_ + margin_) &&
      t.pose.y < (border_max_ - margin_);

    // Hit wall
    if (outside && !t.went_outside) {
      t.went_outside = true;
      t.doing_turn = true;
      t.wait_for_new_input = true;

      // approx 180-degree turn
      t.turn_ticks = 105;

      RCLCPP_WARN(this->get_logger(),
        "%s touched border. Turning 180 degrees.", name.c_str());
    }

    if (comfortably_inside && t.went_outside) {
      t.went_outside = false;
    }

    if (t.doing_turn && t.turn_ticks > 0) {
      cmd.linear.x = 0;
      cmd.angular.z = 1.5;  // fixed turning speed
      t.turn_ticks--;

      if (t.turn_ticks == 0) {
        t.doing_turn = false;
        cmd = geometry_msgs::msg::Twist();
      }
    }
    else if (t.wait_for_new_input) {
      cmd = geometry_msgs::msg::Twist();
    }
  }

  // Push turtles apart briefly
  void runSeparation(geometry_msgs::msg::Twist & a, geometry_msgs::msg::Twist & b)
  {
    a.linear.x = 1.0;
    a.angular.z = -1.0;

    b.linear.x = -1.0;
    b.angular.z = 1.0;

    sep_ticks_--;
    if (sep_ticks_ <= 0) {
      separating_ = false;
      a = geometry_msgs::msg::Twist();
      b = geometry_msgs::msg::Twist();
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceNode>());
  rclcpp::shutdown();
  return 0;
}
