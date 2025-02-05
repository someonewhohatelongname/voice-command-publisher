#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <pv_porcupine.h>  
#include <portaudio.h>     
#include <vector>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class VehicleCommandPublisher : public rclcpp::Node {
public:
    VehicleCommandPublisher() : Node("vehicle_command_publisher") {
        publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

       
        voice_thread_ = std::thread(&VehicleCommandPublisher::voiceRecognition, this);
    }

    ~VehicleCommandPublisher() {
        if (voice_thread_.joinable()) {
            voice_thread_.join();
        }
    }

    void sendCommand(float param1, float param2) {
        auto message = px4_msgs::msg::VehicleCommand();
        message.timestamp = this->now().nanoseconds() / 1000;
        message.param1 = param1;
        message.param2 = param2;
        message.param3 = 0.0;
        message.param4 = 0.0;
        message.param5 = 0.0;
        message.param6 = 0.0;
        message.param7 = 0.0;
        message.command = 187;
        message.target_system = 1;
        message.target_component = 1;
        message.source_system = 1;
        message.source_component = 1;
        message.from_external = true;

        publisher_->publish(message);
    }

    void open_actuator() { sendCommand(1.0f, 1.0f); }
    void close_actuator() { sendCommand(-1.0f, -1.0f); }

    void voiceRecognition() {
        const char *access_key = "T0X/oYqY5ha/S5io2KywIoP1r4xfzzULLBHYAQ8YWse3NN1q1Ox5UQ==";
        const char* model_path = "/home/hn/porcupine/lib/common/porcupine_params.pv";  // Path to the Porcupine model file
        const char *keyword_paths[] = {
            "/home/hn/Voice_command/gripper--open_en_linux_v3_0_0.ppn",
            "/home/hn/Voice_command/gripper-close_en_linux_v3_0_0.ppn"
        };
        
        const float sensitivities[] = {0.5f, 0.5f, 0.5f};  // Adjust as needed

        pv_porcupine_t *porcupine;
        pv_status_t status = pv_porcupine_init(access_key, model_path, 2, keyword_paths, sensitivities, &porcupine);
        if (status != PV_STATUS_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Porcupine!");
            return;
        }

        // Initialize PortAudio for real audio input
        Pa_Initialize();
        PaStream *stream;
        Pa_OpenDefaultStream(&stream, 1, 0, paInt16, 16000, 512, NULL, NULL);
        Pa_StartStream(stream);

        std::vector<int16_t> audio_frame(512);

        while (rclcpp::ok()) {
            Pa_ReadStream(stream, audio_frame.data(), 512);

            int32_t keyword_index;
            status = pv_porcupine_process(porcupine, audio_frame.data(), &keyword_index);

            if (status == PV_STATUS_SUCCESS && keyword_index >= 0) {
                if (keyword_index == 0) {
                    RCLCPP_INFO(this->get_logger(), "Gripper Open Command Detected!");
                    open_actuator();
                } else if (keyword_index == 1) {
                    RCLCPP_INFO(this->get_logger(), "Gripper Close Command Detected!");
                    close_actuator();
                }
            }
        }

        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
        pv_porcupine_delete(porcupine);
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
    std::thread voice_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleCommandPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
