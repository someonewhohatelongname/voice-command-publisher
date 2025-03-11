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
    target_yaw_(0.0),
    // Initialize target values to current values:
    target_setpoint_z_(2.0),
    target_x_(0.0),
    target_y_(0.0),
    new_udp_command_(false),
    start_offboard_(false),
    land_and_disarm_(false),
    stop_movement_(false),
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
    timer_ = this->create_wall_timer(50ms, std::bind(&OffboardNode::timer_cb, this));
    
    // Timer for yaw adjustment (every 500ms)
    yaw_timer_ = this->create_wall_timer(500ms, std::bind(&OffboardNode::yaw_adjustment_cb, this));
    
    // Timer for gradual altitude adjustment (every 50ms)
    alt_timer_ = this->create_wall_timer(50ms, std::bind(&OffboardNode::altitude_adjustment_cb, this));
    // Timer for gradual position adjustment (every 500ms)
    pos_timer_ = this->create_wall_timer(300ms, std::bind(&OffboardNode::position_adjustment_cb, this));

    RCLCPP_INFO(this->get_logger(), "MAVROS example node with native UDP receiver started on port %d", udp_port_);
  }

  ~OffboardNode() {
    running_ = false;
    if (udp_thread_.joinable()) {
      udp_thread_.join();
    }
  }

private:
  // Helper function to normalize angle to [-pi, pi]
  double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  // Yaw adjustment callback: adjusts current_yaw_ toward target_yaw_ in small steps.
  void yaw_adjustment_cb() {
    double yaw_error = normalize_angle(target_yaw_ - current_yaw_);
    const double yaw_step = M_PI / 180.0 * 5.0;  // 5° per cycle (~0.0873 rad)
    if (std::fabs(yaw_error) > 0.01) {
      if (std::fabs(yaw_error) < yaw_step)
        current_yaw_ = target_yaw_;
      else if (yaw_error > 0)
        current_yaw_ += yaw_step;
      else
        current_yaw_ -= yaw_step;
      current_yaw_ = normalize_angle(current_yaw_);
    }
  }

  // Altitude adjustment callback: gradually adjusts current_setpoint_z_ toward target_setpoint_z_
  void altitude_adjustment_cb() {
    double error = target_setpoint_z_ - current_setpoint_z_;
    const double alt_step = 0.1; // Adjust altitude by 0.1 m per cycle
    if (std::fabs(error) > 0.01) {
      if (std::fabs(error) < alt_step)
        current_setpoint_z_ = target_setpoint_z_;
      else
        current_setpoint_z_ += (error > 0 ? alt_step : -alt_step);
    }
  }

  // Position adjustment callback: gradually adjusts current_x_ and current_y_ toward target_x_ and target_y_
  void position_adjustment_cb() {
    double error_x = target_x_ - current_x_;
    double error_y = target_y_ - current_y_;
    double distance = std::sqrt(error_x * error_x + error_y * error_y);
    const double pos_step = 0.1; // Adjust position by 0.1 m per cycle
    if (distance > 0.01) {
      double step_ratio = std::min(pos_step / distance, 1.0);
      current_x_ += error_x * step_ratio;
      current_y_ += error_y * step_ratio;
    }
  }

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
            // Reset targets when arming/offboard is commanded.
            target_setpoint_z_ = 2.0;
            target_x_ = current_x_;
            target_y_ = current_y_;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: ARM & OFFBOARD, set altitude target 2.0m");
          } else if (last_udp_cmd_.param1 == 0.0f) {
            land_and_disarm_ = true;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: LAND & DISARM, switch to manual");
          }
        } else if (last_udp_cmd_.command == 178) {
          // Instead of immediate altitude change, update target altitude.
          if (last_udp_cmd_.param2 == -1.0f) {
            target_setpoint_z_ += 0.5;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Increase altitude target by 0.5m, new target: %.2f", target_setpoint_z_);
          } else if (last_udp_cmd_.param2 == 1.0f) {
            target_setpoint_z_ -= 0.5;
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Decrease altitude target by 0.5m, new target: %.2f", target_setpoint_z_);
          }
        } else if (last_udp_cmd_.command == 179) {
          // Yaw command, already handled with gradual adjustment.
          const double yaw_delta = M_PI / 6.0; // 30° adjustment
          if (last_udp_cmd_.param1 == -1.0f) {
            target_yaw_ = normalize_angle(current_yaw_ + yaw_delta);
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Set target yaw left by 30°, target: %.2f rad", target_yaw_);
          } else if (last_udp_cmd_.param1 == 1.0f) {
            target_yaw_ = normalize_angle(current_yaw_ - yaw_delta);
            RCLCPP_INFO(this->get_logger(), "UDP cmd: Set target yaw right by 30°, target: %.2f rad", target_yaw_);
          }
        } else if (last_udp_cmd_.command == 181) {
          // Immediately stop current forward movement by setting target position to current position.
          target_x_ = current_x_;
          target_y_ = current_y_;
          RCLCPP_INFO(this->get_logger(), "UDP cmd: STOP, holding current position (%.2f, %.2f)", current_x_, current_y_);
        } else if (last_udp_cmd_.command == 180) {
          double distance = static_cast<double>(last_udp_cmd_.param1);
          // Update target position based on current yaw.
          target_x_ += distance * std::cos(current_yaw_);
          target_y_ += distance * std::sin(current_yaw_);
          RCLCPP_INFO(this->get_logger(), "UDP cmd: Set target forward %.2f m, new target pos: (%.2f, %.2f)", distance, target_x_, target_y_);
        } else if (last_udp_cmd_.command == 187) {
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

    // Build the setpoint message with updated position and orientation.
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

    // If commanded, switch to OFFBOARD mode and arm.
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
      start_offboard_ = false;
    }

    // Process landing and disarming command.
    if (land_and_disarm_ && (now - last_request_ > rclcpp::Duration(5s))) {
      auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      mode_request->custom_mode = "AUTO.LAND";
      if (set_mode_client_->wait_for_service(1s)) {
        set_mode_client_->async_send_request(mode_request);
        RCLCPP_INFO(this->get_logger(), "Landing mode command sent");
      } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service not available for landing");
      }
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

  // UDP Receiver using native sockets.
  void udp_receiver_thread() {
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
      throw std::runtime_error("Failed to create UDP socket");
    }
    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
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

  void start_udp_receiver() {
    running_ = true;
    udp_thread_ = std::thread(&OffboardNode::udp_receiver_thread, this);
    RCLCPP_INFO(this->get_logger(), "Started native UDP receiver on port %d", udp_port_);
  }

  // ---------------- Member Variables ----------------
  int udp_port_;
  enum { max_length = 1024 };
  std::mutex udp_mutex_;
  UdpCommand last_udp_cmd_;
  bool new_udp_command_;
  bool running_{false};

  double current_setpoint_z_;
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};  // in radians
  double target_yaw_{0.0};   // desired yaw target

  // New target variables for gradual adjustment.
  double target_setpoint_z_;
  double target_x_;
  double target_y_;

  bool start_offboard_;
  bool land_and_disarm_;
  bool stop_movement_; // (unused in new logic)

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr yaw_timer_;
  rclcpp::TimerBase::SharedPtr alt_timer_;
  rclcpp::TimerBase::SharedPtr pos_timer_;
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_request_;
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
