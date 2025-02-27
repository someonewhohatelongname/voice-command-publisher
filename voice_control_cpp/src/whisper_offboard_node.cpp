// export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/Documents/ws_ros2/src/porcupine/lib/linux/x86_64:/home/ubuntu/Documents/ws_ros2/src/whisper.cpp/build/src

// export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/Documents/ws_ros2/src/porcupine/lib/linux/x86_64:/home/ubuntu/Documents/ws_ros2/src/whisper.cpp/build/src

// export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/Documents/ws_ros2/src/porcupine/lib/linux/x86_64:/home/ubuntu/Documents/ws_ros2/src/whisper.cpp/build/src

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

        // Start thread to monitor whisper.cpp output
        whisper_monitor_thread_ = std::thread(&VoiceControlNode::monitor_whisper_output, this);
        whisper_monitor_thread_.detach();  

        RCLCPP_INFO(this->get_logger(), "Monitoring terminal for Whisper.cpp output");

        // Timer for offboard control
        offboard_timer_ = this->create_wall_timer(100ms, [this]() {
            if (offboard_active_) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }
        });
    }

    ~VoiceControlNode() {
        should_exit_ = true;
        if (whisper_monitor_thread_.joinable()) {
            whisper_monitor_thread_.join();
        }
    }

private:
private:
void monitor_whisper_output() {
    RCLCPP_INFO(this->get_logger(), "Starting Whisper.cpp output monitor");
    
    std::string line;
    std::regex heard_pattern("Heard '([^']+)'");
    
    
    std::cin.sync_with_stdio(false);
    
    while (!should_exit_ && rclcpp::ok()) {
        
        if (std::getline(std::cin, line)) {
            RCLCPP_INFO(this->get_logger(), "Read line: %s", line.c_str());
            
            // Check if this is a line we're interested in
            if (line.find("Heard '") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Detected Whisper output: %s", line.c_str());
                
                // Extract the heard text
                std::smatch matches;
                if (std::regex_search(line, matches, heard_pattern) && matches.size() > 1) {
                    std::string transcription = matches[1].str();
                    RCLCPP_INFO(this->get_logger(), "Extracted transcription: '%s'", transcription.c_str());
                    
                    // Process the command
                    process_whisper_output(transcription);
                }
            }
        } else {
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Increased sleep time
        }
    }
    RCLCPP_INFO(this->get_logger(), "Whisper.cpp output monitor stopped");
}

    void process_whisper_output(const std::string& transcription) {
        RCLCPP_INFO(this->get_logger(), "Processing command: %s", transcription.c_str());
        
        // Convert to lowercase for case-insensitive matching
        std::string lower_text = transcription;
        std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);

        // Remove trailing period if present
        std::string processed_text = lower_text;

        processed_text.erase(std::remove(processed_text.begin(), processed_text.end(), '.'), processed_text.end());
        

        RCLCPP_INFO(this->get_logger(), "Processed command (after lowercase and period removal): '%s'", processed_text.c_str());

        // Check for commands in the transcription (using find () , etc drone alpha arm also can arm)
        if (processed_text.find("drone alpha") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Arm");
            armDrone();
        }
        else if (processed_text.find("disarm") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Disarm");
            disarmDrone();
        }
        else if (processed_text.find("offboard") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Offboard");
            switchToOffboardMode();
        }
        else if (processed_text.find("gripper open") != std::string::npos || processed_text.find("open gripper") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Open Gripper");

             sendGripperCommand(1.0f, 1.0f);
        }
        else if (processed_text.find("gripper close") != std::string::npos || processed_text.find("close gripper") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Close Gripper");
            
            // Keeping the function for future use
            // sendGripperCommand(-1.0f, -1.0f);
        }
        else if (processed_text.find("move forward") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Move Forward");
            // Implement move forward logic here
        }
        else if (processed_text.find("yaw left") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Voice command detected: Yaw Left");
            // Implement yaw left logic here
        }
        
        // else {
        //     RCLCPP_INFO(this->get_logger(), "Unrecognized command: %s", transcription.c_str());
        // }
    }

    // Keeping this function for future use
    void sendGripperCommand(float param1, float param2) {
        

        auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request.param1 = param1;
        request->request.param2 = param2;
        request->request.command = 187;  // Command specific to gripper
        request->request.target_system = 1;
        request->request.target_component = 1;
        request->request.from_external = true;
        
        gripper_client_->async_send_request(request);
        
        
        RCLCPP_INFO(this->get_logger(), "Gripper command parameters (not sent): param1=%.1f, param2=%.1f", param1, param2);
    }

    void armDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent to PX4.");
        
        if (offboard_active_) {
            startMissionSequence();
        }
    }

    void disarmDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent to PX4.");
    }

    void switchToOffboardMode() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        RCLCPP_INFO(this->get_logger(), "Offboard mode command sent to PX4.");

        std::this_thread::sleep_for(2s);
        armDrone();

        offboard_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Offboard control activated.");

        startMissionSequence();
    }

    void startMissionSequence() {
        RCLCPP_INFO(this->get_logger(), "Hovering for 5 seconds...");
        auto start_time = this->now();
        while ((this->now() - start_time).seconds() < 10) {
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            RCLCPP_DEBUG(this->get_logger(), "Publishing setpoints...");
            std::this_thread::sleep_for(20ms);
        }

        landDrone();
        RCLCPP_INFO(this->get_logger(), "Land command sent. Waiting for the drone to land...");

        std::this_thread::sleep_for(20s);

        RCLCPP_INFO(this->get_logger(), "Disarm command sent.");
        
        std::this_thread::sleep_for(4s);
        
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
        RCLCPP_INFO(this->get_logger(), "Manual mode command sent to PX4.");
        
        offboard_active_ = false;
    }

    void landDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Land command sent to PX4.");
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
        RCLCPP_DEBUG(this->get_logger(), "Offboard control mode published.");
    }

    void publish_trajectory_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {0.0, 0.0, -1.0};
        msg.yaw = 0.0;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Trajectory setpoint published: [%.2f, %.2f, %.2f]", 
                     msg.position[0], msg.position[1], msg.position[2]);
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
        RCLCPP_INFO(this->get_logger(), "Vehicle command sent: command=%d, param1=%.2f, param2=%.2f", 
                    command, param1, param2);
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