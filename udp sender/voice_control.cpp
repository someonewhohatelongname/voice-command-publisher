/*
Updates-->
Added function to call two drones 
*/

#include <iostream>
#include <string>
#include <thread>
#include <regex>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <wiringPi.h>

#define BUTTON_PIN 27  // GPIO Pin for physical stop button

struct VehicleCommand {
    uint16_t command;
    float param1;
    float param2;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;
    uint8_t source_component;
    bool from_external;

    void pack(uint8_t* buffer) {
        std::memcpy(buffer, this, sizeof(VehicleCommand));
    }
};

class VoiceControl {
public:
    VoiceControl(const std::string& udp_ip_alpha, int udp_port_alpha, 
                 const std::string& udp_ip_beta, int udp_port_beta)
        : udp_ip_alpha_(udp_ip_alpha), udp_port_alpha_(udp_port_alpha),
          udp_ip_beta_(udp_ip_beta), udp_port_beta_(udp_port_beta),
          active_drone_("alpha") // Default to Alpha
    {
        setup_udp_socket();
        setup_gpio();
        monitor_whisper_output();
    }

    ~VoiceControl() {
        close(udp_socket_);
    }

private:
    std::string udp_ip_alpha_, udp_ip_beta_;
    int udp_port_alpha_, udp_port_beta_;
    std::string active_drone_;
    int udp_socket_;
    std::thread whisper_monitor_thread_;
    std::regex heard_pattern_ = std::regex("Heard '([^']+)'");  // Regex to extract recognized text

    void setup_udp_socket() {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            std::cerr << "Failed to create UDP socket" << std::endl;
            exit(1);
        }
    }

    void send_udp(const VehicleCommand& cmd) {
        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(active_drone_ == "alpha" ? udp_port_alpha_ : udp_port_beta_);
        inet_pton(AF_INET, active_drone_ == "alpha" ? udp_ip_alpha_.c_str() : udp_ip_beta_.c_str(), &server_addr.sin_addr);

        uint8_t buffer[sizeof(VehicleCommand)];
        cmd.pack(buffer);

        ssize_t sent = sendto(udp_socket_, buffer, sizeof(buffer), 0, 
                              (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            std::cerr << "Failed to send UDP packet" << std::endl;
        } else {
            std::cout << "Sent command to " << active_drone_ << ": " << cmd.command << std::endl;
        }
    }

    void setup_gpio() {
        wiringPiSetupGpio();
        pinMode(BUTTON_PIN, INPUT);
        pullUpDnControl(BUTTON_PIN, PUD_UP);
        std::thread([this]() {
            while (true) {
                if (digitalRead(BUTTON_PIN) == LOW) {  // Button pressed
                    std::cout << "GPIO button pressed! Sending stop command." << std::endl;
                    stop_movement();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }).detach();
    }

    void monitor_whisper_output() {
        std::cout << "Starting Whisper.cpp output monitor" << std::endl;
        std::string line;
        std::cin.sync_with_stdio(false);

        while (true) {
            if (std::getline(std::cin, line)) {
                std::cout << "Read line: " << line << std::endl;
                std::smatch matches;
                if (std::regex_search(line, matches, heard_pattern_) && matches.size() > 1) {
                    std::string transcription = matches[1].str();
                    std::cout << "Extracted transcription: " << transcription << std::endl;
                    process_whisper_output(transcription);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    void process_whisper_output(const std::string& transcription) {
        std::string lower_text = transcription;
        std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(), ::tolower);
        lower_text.erase(std::remove(lower_text.begin(), lower_text.end(), '.'), lower_text.end());

        // Change active drone
        if (lower_text.find("drone alpha") != std::string::npos) {
            active_drone_ = "alpha";
            std::cout << "Now controlling: Drone Alpha" << std::endl;
        } else if (lower_text.find("drone beta") != std::string::npos) {
            active_drone_ = "beta";
            std::cout << "Now controlling: Drone Beta" << std::endl;
        }

        // Commands
        else if (lower_text.find("alpha") != std::string::npos) {
            arm_drone();
        } else if (lower_text.find("off board") != std::string::npos) {
            switch_to_offboard_mode();
        } else if (lower_text.find("gripper open") != std::string::npos) {
            open_gripper();
        } else if (lower_text.find("gripper close") != std::string::npos) {
            close_gripper();
        } else if (lower_text.find("go up") != std::string::npos) {
            throttle_up();
        } else if (lower_text.find("go down") != std::string::npos) {
            throttle_down();
        } else if (lower_text.find("turn left") != std::string::npos) {
            yaw_left();
        } else if (lower_text.find("turn right") != std::string::npos) {
            yaw_right();
        } else if (lower_text.find("move forward") != std::string::npos) {
            move_forward();
        } else if (lower_text.find("stop") != std::string::npos) {
            stop_movement();
        }
    }

    void arm_drone() {
        VehicleCommand cmd = {400, 1.0};
        send_udp(cmd);
    }

    void switch_to_offboard_mode() {
        VehicleCommand cmd = {176, 1.0, 6.0};
        send_udp(cmd);
    }

    void open_gripper() {
        VehicleCommand cmd = {187, 1.0, 1.0};
        send_udp(cmd);
    }

    void close_gripper() {
        VehicleCommand cmd = {187, -1.0, -1.0};
        send_udp(cmd);
    }

    void throttle_up() {
        VehicleCommand cmd = {178, 2.0, -1.0};
        send_udp(cmd);
    }

    void throttle_down() {
        VehicleCommand cmd = {178, 2.0, 1.0};
        send_udp(cmd);
    }

    void yaw_left() {
        VehicleCommand cmd = {179, -1.0};
        send_udp(cmd);
    }

    void yaw_right() {
        VehicleCommand cmd = {179, 1.0};
        send_udp(cmd);
    }

    void move_forward() {
        VehicleCommand cmd = {180, 10.0};
        send_udp(cmd);
    }

    void stop_movement() {
        VehicleCommand cmd = {181, 0.0};
        send_udp(cmd);
    }
};

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <udp_ip_alpha> <udp_port_alpha> <udp_ip_beta> <udp_port_beta>" << std::endl;
        return 1;
    }

    try {
        std::string udp_ip_alpha = argv[1];
        int udp_port_alpha = std::stoi(argv[2]);
        std::string udp_ip_beta = argv[3];
        int udp_port_beta = std::stoi(argv[4]);

        VoiceControl voiceControl(udp_ip_alpha, udp_port_alpha, udp_ip_beta, udp_port_beta);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
