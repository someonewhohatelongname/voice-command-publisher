#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_long.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>       // for close()
#include <thread>
#include <mutex>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

// Structure for the UDP command data (17 bytes total)
struct UdpCommand {
  uint32_t command;           // 4 bytes now instead of 2
  float param1;               // 4 bytes
  float param2;               // 4 bytes
  uint8_t target_system;      // 1 byte
  uint8_t target_component;   // 1 byte
  uint8_t source_system;      // 1 byte
  uint8_t source_component;   // 1 byte
  bool from_external;         // 1 byte
} __attribute__((packed));

class OffboardNode : public rclcpp::Node
{
public:
  OffboardNode()
  : Node("mavros_example"),
    current_setpoint_z_(2.0),
    current_x_(0.0),
    current_y_(0.0),
    current_yaw_(0.0),
    new_udp_command_(false),
    start_offboard_(false),
    land_and_disarm_(false),
    last_request_(this->now())
  {
    // Declare and get the UDP port as a parameter. Default is 9000.
    this->declare_parameter<int>("udp_port", 9000);
    this->get_parameter("udp_port", udp_port_);

    // Set up ROS2 interfaces:
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&OffboardNode::state_cb, this, std::placeholders::_1));

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    // Client for sending arbitrary MAVLink commands.
    command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");

    // Start the UDP receiver thread (using native socket API)
    start_udp_receiver();

    // Timer for main control loop (20 Hz)
    timer_ = this->create_wall_timer(
      50ms, std::bind(&OffboardNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "MAVROS example node with native UDP receiver started on port %d", udp_port_);
  }

  ~OffboardNode() {
    running_ = false;
    if (udp_thread_.joinable()) {
      udp_thread_.join();
    }
  }

private:
  // Callback for receiving vehicle state updates.
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  // Main timer callback (20 Hz)
  void timer_cb()
  {
    // Process any new UDP command (thread-safe)
    {
      std::lock_guard<std::mutex> lock(udp_mutex_);
      if (new_udp_command_) {
        // Process commands based on our defined protocol:
        if (last_udp_cmd_.command == 400) {
          if (last_udp_cmd_.param1 == 1.0f) {
            start_offboard_ = true;
            current_setpoint_z_ = 2.0; // Reset altitude to 2.0 m
            RCLCPP_INFO(this->get_logger(), "UDP cmd: ARM & OFFBOARD, set altitude 2.0m");
          } else if (last_udp_cmd_.param1 == 0.0f) {
            land_and_disarm_ = true;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: LAND & DISARM, switch to manual");
          }
        } else if (last_udp_cmd_.command == 178) {
          if (last_udp_cmd_.param2 == -1.0f) {
            // Increase altitude by 0.5 m (adjusting z upward)
            current_setpoint_z_ += 0.5;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Increase altitude by 0.5m, new setpoint: %.2f", current_setpoint_z_);
          } else if (last_udp_cmd_.param2 == 1.0f) {
            // Decrease altitude by 0.5 m (adjusting z downward)
            current_setpoint_z_ -= 0.5;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Decrease altitude by 0.5m, new setpoint: %.2f", current_setpoint_z_);
          }
        } else if (last_udp_cmd_.command == 179) {
          // Yaw commands: adjust current yaw by 30 degrees (pi/6 radians)
          const double yaw_delta = M_PI / 6.0;
          if (last_udp_cmd_.param1 == -1.0f) {
            current_yaw_ += yaw_delta;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Yaw left by 30°, new yaw: %.2f rad", current_yaw_);
          } else if (last_udp_cmd_.param1 == 1.0f) {
            current_yaw_ -= yaw_delta;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Yaw right by 30°, new yaw: %.2f rad", current_yaw_);
          }
        } else if (last_udp_cmd_.command == 180) {
          // Move forward: param1 holds the distance in meters.
          double distance = static_cast<double>(last_udp_cmd_.param1);
          // Update x and y based on the current yaw (forward vector)
          current_x_ += distance * std::cos(current_yaw_);
          current_y_ += distance * std::sin(current_yaw_);
          RCLCPP_INFO(this->get_logger(), "UDP cmd: Move forward %.2f m, new pos: (%.2f, %.2f)", 
                      distance, current_x_, current_y_);
        } else if (last_udp_cmd_.command == 187) {
            // Gripper command:
            // Open gripper when param1==1 and param2==1,
            // Close gripper when param1==-1 and param2==-1.
            auto cmd_long_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
            cmd_long_request->command = 187;
            if (last_udp_cmd_.param1 == 1.0f && last_udp_cmd_.param2 == 1.0f) {
                cmd_long_request->param1 = 1.0f;
                cmd_long_request->param2 = 1.0f;
                RCLCPP_INFO(this->get_logger(), "UDP cmd: Gripper OPEN");
            } else if (last_udp_cmd_.param1 == -1.0f && last_udp_cmd_.param2 == -1.0f) {
                cmd_long_request->param1 = -1.0f;
                cmd_long_request->param2 = -1.0f;
                RCLCPP_INFO(this->get_logger(), "UDP cmd: Gripper CLOSE");
            }
            // Set remaining parameters to 0
            cmd_long_request->param3 = 0.0f;
            cmd_long_request->param4 = 0.0f;
            cmd_long_request->param5 = 0.0f;
            cmd_long_request->param6 = 0.0f;
            cmd_long_request->param7 = 0.0f;
            if (command_client_->wait_for_service(1s)) {
                command_client_->async_send_request(cmd_long_request);
            } else {
                RCLCPP_WARN(this->get_logger(), "CommandLong service not available for gripper control");
            }
        }
        new_udp_command_ = false;
      }
    }

    // Build the setpoint message with updated x, y, z and yaw orientation.
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = this->get_clock()->now();
    pose.pose.position.x = current_x_;
    pose.pose.position.y = current_y_;
    pose.pose.position.z = current_setpoint_z_;
    
    // Convert current yaw to quaternion.
    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw_);
    pose.pose.orientation = tf2::toMsg(q);
    
    setpoint_pub_->publish(pose);

    auto now = this->now();

    // If commanded, switch to OFFBOARD mode and arm
    if (start_offboard_ && current_state_.mode != "OFFBOARD" &&
        (now - last_request_ > rclcpp::Duration(5s))) {
      auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_request->custom_mode = "OFFBOARD";
      if (set_mode_client_->wait_for_service(1s)) {
        set_mode_client_->async_send_request(mode_request);
        RCLCPP_INFO(this->get_logger(), "OFFBOARD mode command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service not available");
      }
      last_request_ = now;
    }
    else if (start_offboard_ && !current_state_.armed &&
             (now - last_request_ > rclcpp::Duration(5s))) {
      auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      arm_request->value = true;
      if (arming_client_->wait_for_service(1s)) {
        arming_client_->async_send_request(arm_request);
        RCLCPP_INFO(this->get_logger(), "Arming command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "Arming service not available");
      }
      last_request_ = now;
      start_offboard_ = false; // Reset flag after issuing commands.
    }

    // Process landing and disarming command:
    if (land_and_disarm_ && (now - last_request_ > rclcpp::Duration(5s))) {
      // Switch to landing mode
      auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_request->custom_mode = "AUTO.LAND";
      if (set_mode_client_->wait_for_service(1s)) {
        set_mode_client_->async_send_request(mode_request);
        RCLCPP_INFO(this->get_logger(), "Landing mode command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service not available for landing");
      }
      // Then disarm the vehicle
      auto disarm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      disarm_request->value = false;
      if (arming_client_->wait_for_service(1s)) {
        arming_client_->async_send_request(disarm_request);
        RCLCPP_INFO(this->get_logger(), "Disarming command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "Arming service not available for disarming");
      }
      last_request_ = now;
      land_and_disarm_ = false;
    }
  }

  // ---------------- UDP Receiver using Native Sockets ----------------
  // This thread function creates a UDP socket and continuously waits for incoming packets.
  void udp_receiver_thread() {
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
      throw std::runtime_error("Failed to create UDP socket");
    }

    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
    local_addr.sin_port = htons(udp_port_);

    if (bind(sock_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
      close(sock_fd);
      throw std::runtime_error("Failed to bind UDP socket");
    }

    char buffer[max_length];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (rclcpp::ok() && running_) {
      ssize_t n = recvfrom(sock_fd, buffer, max_length, 0, (struct sockaddr *)&sender_addr, &sender_len);
      if (n < 0) {
        std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
        continue;
      }
      if (n == sizeof(UdpCommand)) {
        UdpCommand cmd;
        std::memcpy(&cmd, buffer, sizeof(UdpCommand));
        {
          std::lock_guard<std::mutex> lock(udp_mutex_);
          last_udp_cmd_ = cmd;
          new_udp_command_ = true;
        }
      } else {
        std::cerr << "Unexpected UDP packet size: " << n << " bytes" << std::endl;
      }
    }
    close(sock_fd);
  }

  // Starts the UDP receiver thread.
  void start_udp_receiver() {
    running_ = true;
    udp_thread_ = std::thread(&OffboardNode::udp_receiver_thread, this);
    RCLCPP_INFO(this->get_logger(), "Started native UDP receiver on port %d", udp_port_);
  }

  // ---------------- Member Variables ----------------

  // UDP parameters
  int udp_port_;
  enum { max_length = 1024 };

  std::mutex udp_mutex_;
  UdpCommand last_udp_cmd_;
  bool new_udp_command_;
  bool running_{false};

  // Position and orientation state
  double current_setpoint_z_;
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};  // in radians

  // Flags for actions
  bool start_offboard_;
  bool land_and_disarm_;

  // ROS interfaces
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_request_;

  // UDP thread
  std::thread udp_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
