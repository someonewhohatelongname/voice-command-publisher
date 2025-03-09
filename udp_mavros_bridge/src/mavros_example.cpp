#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node
{
public:
  OffboardNode()
  : Node("offboard_node"),
    last_request_(this->now())
  {
    // Subscribe to the vehicle state published by MAVROS
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&OffboardNode::state_cb, this, std::placeholders::_1));

    // Publisher for position setpoints (local frame)
    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

    // Service clients for arming and mode switching
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    // Timer to publish setpoints and check state at 20 Hz
    timer_ = this->create_wall_timer(
      50ms, std::bind(&OffboardNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "Offboard node started.");
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void timer_cb()
  {
    // Prepare a steady setpoint to hover at 2 meters (in ENU frame)
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = this->get_clock()->now();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 2.0;
    setpoint_pub_->publish(pose);

    // Check if we need to switch to OFFBOARD mode or arm the vehicle
    auto now = this->now();
    // Try switching to OFFBOARD mode every 5 seconds if not already in it.
    if (current_state_.mode != "OFFBOARD" && (now - last_request_ > rclcpp::Duration(5s))) {
      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->custom_mode = "OFFBOARD";
      if (set_mode_client_->wait_for_service(1s)) {
        auto future_result = set_mode_client_->async_send_request(request);
        // (Optionally, you can wait for the result or add a callback to confirm mode change.)
        RCLCPP_INFO(this->get_logger(), "OFFBOARD mode command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service not available");
      }
      last_request_ = now;
    }
    // Once in OFFBOARD mode, try arming if not armed yet.
    else if (!current_state_.armed && (now - last_request_ > rclcpp::Duration(5s))) {
      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = true;
      if (arming_client_->wait_for_service(1s)) {
        auto future_result = arming_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Arming command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "Arming service not available");
      }
      last_request_ = now;
    }
  }

  // ROS2 interfaces
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_request_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
