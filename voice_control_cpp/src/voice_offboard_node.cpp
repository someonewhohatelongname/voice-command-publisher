#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pv_porcupine.h>
#include <portaudio.h>
#include <vector>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <cstring>

// Define the VehicleCommand structure
struct VehicleCommand {
    uint16_t command;
    float param1;
    float param2 = 0.0;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;
    uint8_t source_component;
    bool from_external;
};

class VoiceControlNode {
public:
    VoiceControlNode(const char* udp_ip, int udp_port) : udp_ip_(udp_ip), udp_port_(udp_port) {
        // Initialize UDP socket
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket");
        }

        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(udp_port_);
        inet_pton(AF_INET, udp_ip_, &server_addr_.sin_addr);

        // Initialize Porcupine for wake word detection
        const char* access_key = "/CtsYopLA1PZVnMC2ltd+Qmi4gFlmMIVNDdnB1uj0tccOyUIT/ssUg==";
        const char* model_path = "/home/raspberry/porcupine/lib/common/porcupine_params.pv";
        const char* keyword_paths[] = {
            "/home/raspberry/voice-command-publisher/Drone-alpha_en_raspberry-pi_v3_0_0.ppn",
            "/home/raspberry/voice-command-publisher/Go-disarm_en_raspberry-pi_v3_0_0.ppn",
            "/home/raspberry/voice-command-publisher/mode-off-board_en_raspberry-pi_v3_0_0.ppn"
            //"/home/raspberry/voice-command-publisher/gripper--open_en_linux_v3_0_0.ppn",
            //"/home/raspberry/voice-command-publisher/gripper-close_en_linux_v3_0_0.ppn"
        };
        const float sensitivities[] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

        pv_status_t status = pv_porcupine_init(
            access_key,
            model_path,
            5,
            keyword_paths,
            sensitivities,
            &porcupine_
        );

        if (status != PV_STATUS_SUCCESS) {
            std::cerr << "Porcupine initialization failed with status: " << status << std::endl;
            throw std::runtime_error("Failed to initialize Porcupine.");
        }

        std::cout << "Porcupine initialized successfully." << std::endl;

        // Initialize PortAudio
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            std::cerr << "PortAudio initialization failed: " << Pa_GetErrorText(err) << std::endl;
            throw std::runtime_error("PortAudio initialization failed.");
        }

        err = Pa_OpenDefaultStream(
            &audio_stream_,
            1,
            0,
            paInt16,
            pv_sample_rate(),
            pv_porcupine_frame_length(),
            audioCallback,
            this
        );
        
        if (err != paNoError) {
            std::cerr << "Failed to open audio stream: " << Pa_GetErrorText(err) << std::endl;
            throw std::runtime_error("Failed to open audio stream.");
        }

        err = Pa_StartStream(audio_stream_);
        if (err != paNoError) {
            std::cerr << "Failed to start audio stream: " << Pa_GetErrorText(err) << std::endl;
            throw std::runtime_error("Failed to start audio stream.");
        }

        std::cout << "Audio stream initialized successfully." << std::endl;
    }

    ~VoiceControlNode() {
        Pa_StopStream(audio_stream_);
        Pa_CloseStream(audio_stream_);
        Pa_Terminate();
        pv_porcupine_delete(porcupine_);
        //close(sock_fd_);
    }

    void run() {
        std::cout << "Voice control running. Press Ctrl+C to exit." << std::endl;
        while (true) {
            Pa_Sleep(100); // Sleep to prevent busy waiting
        }
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
            std::cerr << "Error processing audio." << std::endl;
            return paContinue;
        }

        if (keyword_index >= 0) {
            std::cout << "Wake word detected! Index: " << keyword_index << std::endl;
            node->handle_voice_command(keyword_index);
        }

        return paContinue;
    }

    void handle_voice_command(int32_t keyword_index) {
        switch (keyword_index) {
            case 0:  // "arm"
                std::cout << "Voice command: Arm" << std::endl;
                armDrone();
                break;
            case 1:  // "disarm"
                std::cout << "Voice command: Disarm" << std::endl;
                disarmDrone();
                break;
            case 2:  // "offboard"
                std::cout << "Voice command: Offboard" << std::endl;
                //switchToOffboardMode();
                break;
            case 3:  // "gripper open"
                std::cout << "Voice command: Open Gripper" << std::endl;
                sendGripperCommand(1.0f, 1.0f);
                break;
            case 4:  // "gripper close"
                std::cout << "Voice command: Close Gripper" << std::endl;
                sendGripperCommand(-1.0f, -1.0f);
                break;
            default:
                std::cerr << "Unknown wake word index: " << keyword_index << std::endl;
                break;
        }
    }

    void sendGripperCommand(float param1, float param2) {
        VehicleCommand cmd{};
        cmd.command = 187;  // Command specific to gripper
        cmd.param1 = param1;
        cmd.param2 = param2;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;

        ssize_t sent = sendto(sock_fd_, &cmd, sizeof(cmd), 0, 
                              (struct sockaddr*)&server_addr_, sizeof(server_addr_));
        
        if (sent < 0) {
            std::cerr << "Failed to send UDP message: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Gripper command sent via UDP." << std::endl;
        }
    }

    void armDrone() {
        VehicleCommand cmd{};
        cmd.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0f;  // 1 to arm, 0 to disarm
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;

        ssize_t sent = sendto(sock_fd_, &cmd, sizeof(cmd), 0, 
                              (struct sockaddr*)&server_addr_, sizeof(server_addr_));
        
        if (sent < 0) {
            std::cerr << "Failed to send UDP message: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Arm command sent via UDP." << std::endl;
        }
    }

    void disarmDrone() {
        VehicleCommand cmd{};
        cmd.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 0.0f;  // 1 to arm, 0 to disarm
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;

        ssize_t sent = sendto(sock_fd_, &cmd, sizeof(cmd), 0, 
                              (struct sockaddr*)&server_addr_, sizeof(server_addr_));
        
        if (sent < 0) {
            std::cerr << "Failed to send UDP message: " << strerror(errno) << std::endl;
        } else {
            std::cout << "Disarm command sent via UDP." << std::endl;
        }
    }

    pv_porcupine_t* porcupine_;
    PaStream* audio_stream_;
    int sock_fd_;
    struct sockaddr_in server_addr_;
    const char* udp_ip_;
    int udp_port_;
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <udp_ip> <udp_port>" << std::endl;
        return 1;
    }
    try {
        VoiceControlNode controller(argv[1], std::stoi(argv[2]));
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
