#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <pv_porcupine.h>  // Porcupine library
#include <portaudio.h>     // PortAudio for audio capture
#include <vector>
#include <cstdint>
#include <fstream>
#include <cmath>
#include <chrono>  // Add this for chrono literals

using namespace std::chrono_literals;  // Add this for ms literal

class VoiceControlNode : public rclcpp::Node {
public:
    VoiceControlNode() : Node("voice_control_node") {
        // Publisher for sending commands to PX4
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        // Subscriber for vehicle global position
        vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", 10,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                vehicle_global_position_ = *msg;
            });

        // Initialize Porcupine with your Access Key and custom wake words
        const char* access_key = "T0X/oYqY5ha/S5io2KywIoP1r4xfzzULLBHYAQ8YWse3NN1q1Ox5UQ==";  // Replace with your Picovoice Access Key
        const char* model_path = "/home/hn/porcupine/lib/common/porcupine_params.pv";  // Path to the Porcupine model file

        // Paths to wake word files
        const char* keyword_paths[] = {
            "/home/hn/Voice_command/Drone-Alpha_en_linux_v3_0_0.ppn",       // Wake word for "arm"
            "/home/hn/Voice_command/go-disarm_en_linux_v3_0_0.ppn",    // Wake word for "disarm"
            "/home/hn/Voice_command/go-off-board_en_linux_v3_0_0.ppn"   // Wake word for "offboard"
        };

        // Sensitivity values for each wake word
        const float sensitivities[] = {0.5f, 0.5f, 0.5f};  // Adjust as needed

        // Initialize Porcupine with multiple wake words
        pv_status_t status = pv_porcupine_init(
            access_key,
            model_path,
            3,  // Number of keywords
            keyword_paths,  // Array of keyword file paths
            sensitivities,  // Sensitivities array
            &porcupine_  // Porcupine handle
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

        // Open the default audio stream
        Pa_OpenDefaultStream(
            &audio_stream_,
            1,  // Number of input channels
            0,  // Number of output channels
            paInt16,  // Sample format
            pv_sample_rate(),  // Sample rate
            pv_porcupine_frame_length(),  // Frame length
            audioCallback,
            this
        );
        Pa_StartStream(audio_stream_);
        RCLCPP_INFO(this->get_logger(), "Audio stream initialized successfully.");

        // Timer for offboard control
        offboard_timer_ = this->create_wall_timer(100ms, [this]() {
            if (offboard_active_) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }
        });
    }

    ~VoiceControlNode() {
        // Clean up resources
        Pa_StopStream(audio_stream_);
        Pa_CloseStream(audio_stream_);
        Pa_Terminate();
        pv_porcupine_delete(porcupine_);
        RCLCPP_INFO(this->get_logger(), "Porcupine resources released.");
    }

private:
    static int audioCallback(const void* inputBuffer, void* outputBuffer,
                             unsigned long framesPerBuffer,
                             const PaStreamCallbackTimeInfo* timeInfo,
                             PaStreamCallbackFlags statusFlags,
                             void* userData) {
        (void)outputBuffer;  // Explicitly mark as unused
        (void)framesPerBuffer;  // Explicitly mark as unused
        (void)timeInfo;  // Explicitly mark as unused
        (void)statusFlags;  // Explicitly mark as unused

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
                armDrone();
                break;
            case 1:  // "disarm"
                RCLCPP_INFO(this->get_logger(), "Voice command: Disarm");
                disarmDrone();
                break;
            case 2:  // "offboard"
                RCLCPP_INFO(this->get_logger(), "Voice command: Offboard");
                switchToOffboardMode();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown wake word index: %d", keyword_index);
                break;
        }
    }

    void armDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent to PX4.");
        
         if (offboard_active_) {
        startMissionSequence();  // Restart the mission sequence
    }
 }

void disarmDrone() {
    // Step 1: Send the disarm command
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent to PX4.");


}

void switchToOffboardMode() {
    // Step 1: Arm the drone
    //armDrone();
    //RCLCPP_INFO(this->get_logger(), "Arming the drone...");

    // Step 2: Wait for the drone to fully arm
    //std::this_thread::sleep_for(5s);  // Increased delay to ensure arming is complete
    //RCLCPP_INFO(this->get_logger(), "Drone is armed.");

    // Step 3: Send the offboard mode command
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent to PX4.");

    // Step 4: Wait for the drone to switch to offboard mode
    std::this_thread::sleep_for(2s);
    //RCLCPP_INFO(this->get_logger(), "Drone should now be in offboard mode.");
    
    armDrone();

    // Step 5: Activate offboard control
    offboard_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Offboard control activated.");

    // Step 6: Start the mission sequence
    startMissionSequence();
}

    void startMissionSequence() {
        // Hover for 5 seconds
        RCLCPP_INFO(this->get_logger(), "Hovering for 5 seconds...");
        auto start_time = this->now();
        while ((this->now() - start_time).seconds() < 10) {
            // Publish offboard control mode and setpoints continuously
            publish_offboard_control_mode();
            publish_trajectory_setpoint();
            RCLCPP_DEBUG(this->get_logger(), "Publishing setpoints...");
            std::this_thread::sleep_for(20ms);  // 50 Hz
        }

        // Land the drone
        landDrone();
        RCLCPP_INFO(this->get_logger(), "Land command sent. Waiting for the drone to land...");

        // Wait for the drone to land
        std::this_thread::sleep_for(15s);  // Increased delay to 10 seconds

        // Disarm the drone
        disarmDrone();
        RCLCPP_INFO(this->get_logger(), "Disarm command sent.");
        

    }

    void landDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Land command sent to PX4.");
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;  // Enable position control
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
        msg.position = {0.0, 0.0, -5.0};  // Hover at 5 meters (NED frame)
        msg.yaw = -3.14;  // Yaw angle of 180 degrees
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Trajectory setpoint published: [%.2f, %.2f, %.2f]", msg.position[0], msg.position[1], msg.position[2]);
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

        // Log the command being sent
        RCLCPP_INFO(this->get_logger(), "Vehicle command sent: command=%d, param1=%.2f, param2=%.2f", command, param1, param2);
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

    pv_porcupine_t* porcupine_;
    PaStream* audio_stream_;

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
