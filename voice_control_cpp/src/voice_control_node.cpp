#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <pv_porcupine.h>  // Porcupine library
#include <portaudio.h>     // PortAudio for audio capture
#include <vector>
#include <cstdint>
#include <fstream>

class VoiceControlNode : public rclcpp::Node {
public:
    VoiceControlNode() : Node("voice_control_node") {
        // Publisher for sending commands to PX4
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Initialize Porcupine with your Access Key and custom wake word
        const char* access_key = "T0X/oYqY5ha/S5io2KywIoP1r4xfzzULLBHYAQ8YWse3NN1q1Ox5UQ==";  // Replace with your Picovoice Access Key
        const char* model_path = "/home/hn/porcupine/lib/common/porcupine_params.pv";  // Path to the Porcupine model file
        const char* keyword_paths[] = {"/home/hn/Voice_command/Drone-Alpha_en_linux_v3_0_0.ppn"};  // Array of keyword file paths
        const float sensitivity = 0.5f;  // Sensitivity value

        // Initialize Porcupine
        pv_status_t status = pv_porcupine_init(
            access_key,
            model_path,
            1,  // Number of keywords
            keyword_paths,  // Array of keyword file paths
            &sensitivity,  // Sensitivities array
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

        // Redirect PortAudio logs to a file


        Pa_OpenDefaultStream(
            &audio_stream_,
            1,  // Number of input channels
            0,  // Number of output channels
            paInt16,  // Sample format
            pv_sample_rate(),  // Sample rate (use pv_sample_rate() instead of pv_porcupine_sample_rate())
            pv_porcupine_frame_length(),  // Frame length
            audioCallback,
            this
        );
        Pa_StartStream(audio_stream_);
        RCLCPP_INFO(this->get_logger(), "Audio stream initialized successfully.");
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
        auto* node = static_cast<VoiceControlNode*>(userData);
        const auto* pcm = static_cast<const int16_t*>(inputBuffer);

        int32_t keyword_index;
        pv_status_t status = pv_porcupine_process(node->porcupine_, pcm, &keyword_index);
        if (status != PV_STATUS_SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Error processing audio.");
            return paContinue;
        }

        if (keyword_index >= 0) {
            RCLCPP_INFO(node->get_logger(), "Wake word detected! Arming drone...");
            node->armDrone();
        }

        return paContinue;
    }

    void armDrone() {
        // Create a VehicleCommand message to arm the drone
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0;   // 1 to arm, 0 to disarm
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
        msg.from_external = true;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Arm command sent to PX4.");
    }

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    pv_porcupine_t* porcupine_;
    PaStream* audio_stream_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoiceControlNode>());
    rclcpp::shutdown();
    return 0;
}
