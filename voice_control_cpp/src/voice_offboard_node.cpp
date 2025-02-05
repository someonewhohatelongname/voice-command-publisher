#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <pv_porcupine.h>
#include <portaudio.h>
#include <vector>
#include <cstdint>
#include <fstream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class VoiceControlNode : public rclcpp::Node {
public:
    VoiceControlNode() : Node("voice_control_node") {
        // Publishers from original offboard node
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        // New gripper service client
        gripper_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

        // Wait for gripper service
        while (!gripper_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for gripper service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Gripper service not available, waiting...");
        }

        // Global position subscriber
        vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", 10,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                vehicle_global_position_ = *msg;
            });

        // Initialize Porcupine with expanded wake words
        const char* access_key = "T0X/oYqY5ha/S5io2KywIoP1r4xfzzULLBHYAQ8YWse3NN1q1Ox5UQ==";
        const char* model_path = "/home/hn/porcupine/lib/common/porcupine_params.pv";

        // Updated keyword paths to include gripper commands
        const char* keyword_paths[] = {
            "/home/hn/Voice_command/Drone-Alpha_en_linux_v3_0_0.ppn",    // arm
            "/home/hn/Voice_command/go-disarm_en_linux_v3_0_0.ppn",      // disarm
            "/home/hn/Voice_command/go-off-board_en_linux_v3_0_0.ppn",   // offboard
            "/home/hn/Voice_command/gripper--open_en_linux_v3_0_0.ppn",  // gripper open
            "/home/hn/Voice_command/gripper-close_en_linux_v3_0_0.ppn"   // gripper close
        };

        const float sensitivities[] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

        pv_status_t status = pv_porcupine_init(
            access_key,
            model_path,
            5,  // Updated number of keywords
            keyword_paths,
            sensitivities,
            &porcupine_
        );

        if (status != PV_STATUS_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Porcupine initialization failed with status: %d", status);
            throw std::runtime_error("Failed to initialize Porcupine.");
        }
        RCLCPP_INFO(this->get_logger(), "Porcupine initialized successfully.");

        // Initialize PortAudio
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio initialization failed: %s", Pa_GetErrorText(err));
            throw std::runtime_error("PortAudio initialization failed.");
        }

        Pa_OpenDefaultStream(
            &audio_stream_,
            1,
            0,
            paInt16,
            pv_sample_rate(),
            pv_porcupine_frame_length(),
            audioCallback,
            this
        );
        Pa_StartStream(audio_stream_);
        RCLCPP_INFO(this->get_logger(), "Audio stream initialized successfully.");

        offboard_timer_ = this->create_wall_timer(100ms, [this]() {
            if (offboard_active_) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }
        });
    }

    ~VoiceControlNode() {
        Pa_StopStream(audio_stream_);
        Pa_CloseStream(audio_stream_);
        Pa_Terminate();
        pv_porcupine_delete(porcupine_);
    }

private:
    static int audioCallback(const void* inputBuffer, void* outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void* userData) {
        (void)outputBuffer;
        (void)framesPerBuffer;
        (void)timeInfo;
        (void)statusFlags;

        auto* node = static_cast<VoiceControlNode*>(userData);
        const auto* pcm = static_cast<const int16_t*>(inputBuffer);

        int32_t keyword_index;
        pv_status_t status = pv_porcupine_process(node->porcupine_, pcm, &keyword_index);
        if (status != PV_STATUS_SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Error processing audio.");
            return paContinue;
        }

        if (keyword_index >= 0) {
            RCLCPP_INFO(node->get_logger(), "Wake word detected! Index: %d", keyword_index);
            node->handle_voice_command(keyword_index);
        }

        return paContinue;
    }

    void handle_voice_command(int32_t keyword_index) {
        switch (keyword_index) {
            case 0:  // "arm"
                RCLCPP_INFO(this->get_logger(), "Voice command: Arm");
                if (!offboard_active_) {
                    armDrone();
                }
                break;
            case 1:  // "disarm"
                RCLCPP_INFO(this->get_logger(), "Voice command: Disarm");
                disarmDrone();
                break;
            case 2:  // "offboard"
                RCLCPP_INFO(this->get_logger(), "Voice command: Offboard");
                switchToOffboardMode();
                break;
            case 3:  // "gripper open"
                RCLCPP_INFO(this->get_logger(), "Voice command: Open Gripper");
                sendGripperCommand(1.0f, 1.0f);
                break;
            case 4:  // "gripper close"
                RCLCPP_INFO(this->get_logger(), "Voice command: Close Gripper");
                sendGripperCommand(-1.0f, -1.0f);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown wake word index: %d", keyword_index);
                break;
        }
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

    // ... [Keep all existing methods from voice_offboard_node-2.cpp unchanged] ...
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
        msg.position = {0.0, 0.0, -5.0};
        msg.yaw = -3.14;
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

    // Porcupine and audio members
    pv_porcupine_t* porcupine_;
    PaStream* audio_stream_;

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
//export LD_LIBRARY_PATH=/home/hn/porcupine/lib/linux/x86_64:$LD_LIBRARY_PATH
