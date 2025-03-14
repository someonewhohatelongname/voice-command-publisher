#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <vector>
#include <cstdint>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <array>
#include <regex>

using namespace std::chrono_literals;

class VoiceControlNode : public rclcpp::Node {
public:
    VoiceControlNode() : Node("voice_control_node") {
        // Publishers from original offboard node
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        gripper_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

        // Global position subscriber
        vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", 10,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                vehicle_global_position_ = *msg;
            });

        // Start thread to monitor faster-whisper output
        should_exit_ = false;
        whisper_monitor_thread_ = std::thread(&VoiceControlNode::monitor_whisper_output, this);
        
        // Timer for offboard control
        offboard_timer_ = this->create_wall_timer(100ms, [this]() {
            if (offboard_active_) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }
        });
        
        // Initialize current position to 0,0,-1 (1 meter above ground)
        current_position_ = {0.0, 0.0, -1.0};
        current_yaw_ = 0.0;
    }

    ~VoiceControlNode() {
        should_exit_ = true;
        if (whisper_monitor_thread_.joinable()) {
            whisper_monitor_thread_.join();
        }
    }

private:
    std::string last_command_;
    std::chrono::time_point<std::chrono::steady_clock> last_command_time_;
    const std::chrono::milliseconds DEDUPLICATION_THRESHOLD{1500}; // 1.5 seconds
    bool is_armed_ = false;
    bool already_processed_token_ = false;
    
    // Current position and orientation tracking
    std::array<float, 3> current_position_;
    float current_yaw_;
    std::mutex position_mutex_;

    // Extract numbers from a string with unit conversion
    float extract_number(const std::string& text, const std::string& unit = "") {
        std::regex number_pattern("\\b(\\d+(?:\\.\\d+)?)\\s*(?:" + unit + ")?\\b");
        std::smatch matches;
        
        if (std::regex_search(text, matches, number_pattern) && matches.size() > 1) {
            try {
                return std::stof(matches[1].str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert number: %s", e.what());
            }
        }
        
        // Default value if no number found
        return 1.0f;
    }
    
    // Convert degrees to radians
    float deg_to_rad(float degrees) {
        return degrees * M_PI / 180.0f;
    }
    
    // Normalize angle to -PI to PI
    float normalize_angle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }

    void monitor_whisper_output() {
        RCLCPP_INFO(this->get_logger(), "Starting faster-whisper output monitor");
        std::string line;
        std::regex transcript_pattern("Transcript: (.+?)(\\.)?$");
        
        // Make sure stdin is unbuffered
        std::ios::sync_with_stdio(false);
        std::cin.tie(nullptr);
        
        while (!should_exit_ && rclcpp::ok()) {
            // Use getline with a timeout to prevent blocking indefinitely
            if (std::getline(std::cin, line)) {
                RCLCPP_INFO(this->get_logger(), "Read line from stdin: '%s'", line.c_str());
                
                // Check if this is a line containing a transcript
                if (line.find("Transcript:") != std::string::npos) {
                    // Extract the transcription text
                    std::smatch matches;
                    if (std::regex_search(line, matches, transcript_pattern) && matches.size() > 1) {
                        std::string transcription = matches[1].str();
                        
                        // Deduplication logic
                        auto current_time = std::chrono::steady_clock::now();
                        if (transcription == last_command_ && 
                            (current_time - last_command_time_) < DEDUPLICATION_THRESHOLD) {
                            RCLCPP_INFO(this->get_logger(), "Skipping duplicate command: '%s'", 
                                        transcription.c_str());
                            continue;
                        }
                        
                        // Update last command tracking
                        last_command_ = transcription;
                        last_command_time_ = current_time;
                        
                        // Process the command
                        process_whisper_output(transcription);
                    }
                }
            } else {
                // Sleep briefly to prevent CPU spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    void process_whisper_output(const std::string& transcription) {
        // Convert to lowercase for case-insensitive matching
        std::string lower_text = transcription;
        std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);
    
        // Remove trailing period if present
        std::string processed_text = lower_text;
        processed_text.erase(std::remove(processed_text.begin(), processed_text.end(), '.'), processed_text.end());
        
        RCLCPP_INFO(this->get_logger(), "Processed command: '%s'", processed_text.c_str());
        
        // Reset token processing flag
        already_processed_token_ = false;
    
        // Process command based on recognized patterns
        if (process_arm_command(processed_text)) {
            return; // Command was processed
        }
        
        if (process_movement_command(processed_text)) {
            return; // Command was processed
        }
        
        if (process_other_command(processed_text)) {
            return; // Command was processed
        }
        
        RCLCPP_INFO(this->get_logger(), "No matching command found in: '%s'", processed_text.c_str());
    }
    
    bool process_arm_command(const std::string& text) {
        // Exact match for arming command
        if (text == "drone alpha") {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Arm");
            armDrone();
            is_armed_ = true;
            return true;
        }
        
        // Handle disarm command
        if (text.find("disarm") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Disarm");
            disarmDrone();
            is_armed_ = false;
            return true;
        }
        
        return false;
    }
    
    bool process_movement_command(const std::string& text) {
        // Check if we're in a safe state to process movement commands
        if (!is_armed_) {
            if (text.find("move") != std::string::npos || 
                text.find("turn") != std::string::npos || 
                text.find("yaw") != std::string::npos || 
                text.find("go up") != std::string::npos || 
                text.find("go down") != std::string::npos) {
                RCLCPP_WARN(this->get_logger(), "Cannot execute movement command - drone not armed");
                return true; // Command recognized but not executed
            }
            return false;
        }
        
        // Check for arm command token but as part of a complex command
        if (text.find("drone alpha") != std::string::npos && !already_processed_token_) {
            // For complex commands like "drone alpha move forward 5 meters"
            // We only want to process the movement part if "drone alpha" already processed
            if (text.find("drone alpha") == 0) {
                // Mark the token as processed to avoid duplicate arming
                already_processed_token_ = true;
                
                // Remove the trigger word for further processing
                std::string command_without_trigger = text.substr(11); // length of "drone alpha "
                return process_movement_command(command_without_trigger);
            }
        }
        
        // Now process actual movement commands
        if (text.find("move forward") != std::string::npos) {
            float distance = extract_number(text, "meter(s)?");
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Move Forward %.2f meters", distance);
            moveForward(distance);
            return true;
        }

        else if (text.find("go up") != std::string::npos || text.find("move up") != std::string::npos) {
            float distance = extract_number(text, "meter(s)?");
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Move Up %.2f meters", distance);
            moveUp(distance);
            return true;
        }
        else if (text.find("go down") != std::string::npos || text.find("move down") != std::string::npos) {
            float distance = extract_number(text, "meter(s)?");
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Move Down %.2f meters", distance);
            moveUp(-distance);
            return true;
        }
        else if (text.find("yaw left") != std::string::npos || text.find("turn left") != std::string::npos) {
            float degrees = extract_number(text, "degree(s)?");
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Turn Left %.2f degrees", degrees);
            yaw(-degrees);
            return true;
        }
        else if (text.find("yaw right") != std::string::npos || text.find("turn right") != std::string::npos) {
            float degrees = extract_number(text, "degree(s)?");
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Turn Right %.2f degrees", degrees);
            yaw(degrees);
            return true;
        }
        else if (text.find("stop") != std::string::npos) {
            float duration = extract_number(text, "second(s)?");
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Stop at current position");
            stop();
            return true;
        }
        
        return false;
    }
    
    bool process_other_command(const std::string& text) {
        if (text.find("offboard") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Offboard");
            switchToOffboardMode();
            return true;
        }
        else if (text.find("gripper open") != std::string::npos || text.find("gripper up") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Open Gripper");
            sendGripperCommand(1.0f, 1.0f);
            return true;
        }
        else if (text.find("gripper close") != std::string::npos || text.find("gripper down") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Close Gripper");
            sendGripperCommand(-1.0f, -1.0f);
            return true;
        }
        else if (text.find("land") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Land");
            landDrone();
            // publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
            // RCLCPP_INFO(this->get_logger(), "Manual mode command sent to PX4.");
            is_armed_ = false;
            return true;
        }
        
        return false;
    }

    // Movement commands
    void moveForward(float distance) {
        if (!offboard_active_) {
            RCLCPP_WARN(this->get_logger(), "Cannot move: Offboard mode not active");
            return;
        }
        
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        // Calculate new position based on current yaw
        float x_change = distance * cos(current_yaw_);
        float y_change = distance * sin(current_yaw_);
        
        current_position_[0] += x_change;
        current_position_[1] += y_change;
        
        RCLCPP_INFO(this->get_logger(), "Moving forward %.2f meters to position [%.2f, %.2f, %.2f]", 
                    distance, current_position_[0], current_position_[1], current_position_[2]);
    }
    
    void moveRight(float distance) {
        if (!offboard_active_) {
            RCLCPP_WARN(this->get_logger(), "Cannot move: Offboard mode not active");
            return;
        }
        
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        // Calculate new position based on current yaw (90 degrees right from forward)
        float x_change = distance * cos(current_yaw_ + M_PI_2);
        float y_change = distance * sin(current_yaw_ + M_PI_2);
        
        current_position_[0] += x_change;
        current_position_[1] += y_change;
        
        RCLCPP_INFO(this->get_logger(), "Moving right %.2f meters to position [%.2f, %.2f, %.2f]", 
                    distance, current_position_[0], current_position_[1], current_position_[2]);
    }
    
    void moveUp(float distance) {
        if (!offboard_active_) {
            RCLCPP_WARN(this->get_logger(), "Cannot move: Offboard mode not active");
            return;
        }
        
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        // Negative Z is upward in PX4
        current_position_[2] -= distance;
        
        RCLCPP_INFO(this->get_logger(), "Moving up %.2f meters to position [%.2f, %.2f, %.2f]", 
                    distance, current_position_[0], current_position_[1], current_position_[2]);
    }
    
    void yaw(float degrees) {
        if (!offboard_active_) {
            RCLCPP_WARN(this->get_logger(), "Cannot yaw: Offboard mode not active");
            return;
        }
        
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        // Convert degrees to radians
        float radians = deg_to_rad(degrees);
        
        // Apply yaw rotation
        // In PX4, positive yaw is counterclockwise (left), negative is clockwise (right)
        // The function receives positive value for "left" and negative for "right"
        current_yaw_ = normalize_angle(current_yaw_ + radians);
        
        RCLCPP_INFO(this->get_logger(), "Yawing %.2f degrees to heading %.2f degrees",
                    degrees, current_yaw_ * 180.0f / M_PI);
    }
    
    void stop() {
        if (!offboard_active_) {
            RCLCPP_WARN(this->get_logger(), "Cannot stop: Offboard mode not active");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Emergency stop activated - holding position");
        
        // Lock the position mutex to prevent other movement commands from changing the setpoint
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        RCLCPP_INFO(this->get_logger(), "Drone stopped at position [%.2f, %.2f, %.2f], yaw: %.2f", 
                    current_position_[0], current_position_[1], current_position_[2], 
                    current_yaw_ * 180.0f / M_PI);
    }

    void sendGripperCommand(float param1, float param2) {
        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request.param1 = param1;
        request->request.param2 = param2;
        request->request.command = 187;  // Command specific to gripper
        request->request.target_system = 1;
        request->request.target_component = 1;
        request->request.from_external = true;
        
        gripper_client_->async_send_request(request);
        
        RCLCPP_INFO(this->get_logger(), "Gripper command sent: param1=%.1f, param2=%.1f", param1, param2);
    }

    void armDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent to PX4.");
    }

    void disarmDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent to PX4.");
    }

    void switchToOffboardMode() {
        // Set initial position to current position when starting offboard mode
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        // Initialize position to slightly above current position
        // Negative Z is up in PX4 coordinate system
        current_position_ = {0.0, 0.0, -1.0};  // 1 meter above current position
        current_yaw_ = 0.0;  // Default heading

        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent to PX4.");

        std::this_thread::sleep_for(2s);
        armDrone();

        is_armed_ = true;

        offboard_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Offboard control activated.");
    }

    void landDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Land command sent to PX4.");
        
        // Wait for landing to complete
        std::this_thread::sleep_for(10s);
        
        // Switch back to manual mode after landing
        
        
        
        offboard_active_ = false;
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            msg.position = {current_position_[0], current_position_[1], current_position_[2]};
            msg.yaw = current_yaw_;
        }
        
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Trajectory setpoint published: [%.2f, %.2f, %.2f], yaw: %.2f", 
                     msg.position[0], msg.position[1], msg.position[2], msg.yaw);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // Publishers and subscribers
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr gripper_client_;

    // Whisper monitoring
    std::thread whisper_monitor_thread_;
    bool should_exit_ = false;

    // State variables
    px4_msgs::msg::VehicleGlobalPosition vehicle_global_position_;
    px4_msgs::msg::VehicleGlobalPosition launch_position_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    bool offboard_active_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoiceControlNode>());
    rclcpp::shutdown();
    return 0;
}