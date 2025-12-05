#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("ui_node")
  {
    pub_t1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel_input", 10);
    pub_t2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel_input", 10);

    RCLCPP_INFO(this->get_logger(),
      "UI node started.\n"
      "You can control turtle1 or turtle2 from this terminal.\n"
      "Type 'q' as turtle name to quit.\n");
  }

  void run()
  {
    while (rclcpp::ok()) {
      std::string name = askTurtleName();
      if (name.empty()) {
        RCLCPP_INFO(this->get_logger(), "Exiting UI node.");
        break;
      }

      double lin = askNumber("Linear velocity: ");
      double ang = askNumber("Angular velocity: ");

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = lin;
      cmd.angular.z = ang;

      auto pub = (name == "turtle1") ? pub_t1_ : pub_t2_;

      RCLCPP_INFO(this->get_logger(),
        "Sending command to %s: lin = %.2f, ang = %.2f (for about 1 second)",
        name.c_str(), lin, ang);

      // publish once, let safety node handle the rest
      pub->publish(cmd);
      rclcpp::sleep_for(1s);

      // send stop
      geometry_msgs::msg::Twist stop_msg;
      pub->publish(stop_msg);
      RCLCPP_INFO(this->get_logger(), "%s stopped.\n", name.c_str());
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t1_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t2_;

  std::string askTurtleName()
  {
    while (true) {
      std::cout << "Which turtle do you want to move? (turtle1 / turtle2 / q to quit): ";
      std::string s;
      if (!(std::cin >> s)) {
        return "";
      }

      if (s == "q" || s == "Q") {
        return "";
      }
      if (s == "turtle1" || s == "turtle2") {
        return s;
      }

      std::cout << "Please type 'turtle1', 'turtle2' or 'q'.\n";
    }
  }

  double askNumber(const std::string & prompt)
  {
    while (true) {
      std::cout << prompt;
      double value;
      if (std::cin >> value) {
        return value;
      }
      std::cout << "Not a number, try again.\n";
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
