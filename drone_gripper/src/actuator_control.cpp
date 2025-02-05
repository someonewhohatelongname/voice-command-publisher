#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"
#include <chrono>
using namespace std::chrono_literals;

class VehicleCommandClient : public rclcpp::Node
{
public:
  VehicleCommandClient()
  : Node("vehicle_command_client")
  {
    client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    // Example usage
    open_actuator();
    rclcpp::sleep_for(2s);  // Wait for 2 seconds before closing
    close_actuator();
  }

  void open_actuator()
  {
    send_vehicle_command(1.0f, 1.0f);
    RCLCPP_INFO(this->get_logger(), "Sent OPEN actuator command.");
  }

  void close_actuator()
  {
    send_vehicle_command(-1.0f, -1.0f);
    RCLCPP_INFO(this->get_logger(), "Sent CLOSE actuator command.");
  }

private:
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr client_;

  void send_vehicle_command(float param1, float param2)
  {
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    
    request->request.timestamp = this->now().nanoseconds() / 1000;
    request->request.param1 = param1;
    request->request.param2 = param2;
    request->request.param3 = 0.0f;
    request->request.param4 = 0.0f;
    request->request.param5 = 0.0f;
    request->request.param6 = 0.0f;
    request->request.param7 = 0.0f;
    request->request.command = 187;
    request->request.target_system = 1;
    request->request.target_component = 1;
    request->request.source_system = 1;
    request->request.source_component = 1;
    request->request.from_external = true;

    auto result_future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleCommandClient>();
  rclcpp::shutdown();
  return 0;
}

