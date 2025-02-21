#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <signal.h>

struct VehicleCommand {
    uint16_t command;
    float param1;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;
    uint8_t source_component;
    bool from_external;
};

class CommandReceiver {
public:
    CommandReceiver(int port) : running_(true) {
        // Create UDP socket
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        // Configure socket address
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
        addr.sin_port = htons(port);

        // Bind socket
        if (bind(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock_fd_);
            throw std::runtime_error("Failed to bind socket");
        }

        std::cout << "UDP receiver listening on port " << port << std::endl;
    }

    ~CommandReceiver() {
        if (sock_fd_ >= 0) {
            close(sock_fd_);
        }
    }

    void run() {
        while (running_) {
            struct sockaddr_in sender_addr;
            socklen_t sender_addr_len = sizeof(sender_addr);
            VehicleCommand cmd;

            // Receive command
            ssize_t received = recvfrom(sock_fd_, &cmd, sizeof(cmd), 0,
                                      (struct sockaddr*)&sender_addr, &sender_addr_len);

            if (received < 0) {
                std::cerr << "Error receiving data" << std::endl;
                continue;
            }

            if (received != sizeof(VehicleCommand)) {
                std::cerr << "Received incomplete packet: " << received << " bytes" << std::endl;
                continue;
            }

            // Get sender's IP address
            char sender_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(sender_addr.sin_addr), sender_ip, INET_ADDRSTRLEN);

            // Print received command details
            std::cout << "\nReceived command from " << sender_ip << ":" << ntohs(sender_addr.sin_port) << std::endl;
            std::cout << "Command ID: " << cmd.command << std::endl;
            std::cout << "Parameter 1: " << cmd.param1 << std::endl;
            std::cout << "Target System: " << (int)cmd.target_system << std::endl;
            std::cout << "Target Component: " << (int)cmd.target_component << std::endl;
            std::cout << "Source System: " << (int)cmd.source_system << std::endl;
            std::cout << "Source Component: " << (int)cmd.source_component << std::endl;
            std::cout << "From External: " << (cmd.from_external ? "true" : "false") << std::endl;

            // Print hex dump of received packet
            std::cout << "Raw packet hex dump: ";
            uint8_t* bytes = reinterpret_cast<uint8_t*>(&cmd);
            for (size_t i = 0; i < sizeof(VehicleCommand); i++) {
                printf("%02X ", bytes[i]);
            }
            std::cout << std::endl;

            // Process command
            processCommand(cmd);
        }
    }

    void stop() {
        running_ = false;
    }

private:
    void processCommand(const VehicleCommand& cmd) {
        switch (cmd.command) {
            case 400:  // MAV_CMD_COMPONENT_ARM_DISARM
                std::cout << "Processing arm/disarm command: " 
                          << (cmd.param1 > 0.5f ? "ARMING" : "DISARMING") 
                          << std::endl;
                // Here you would implement the actual command forwarding to PX4
                break;

            default:
                std::cout << "Unknown command received: " << cmd.command << std::endl;
                break;
        }
    }

    int sock_fd_;
    bool running_;
};

// Signal handler for graceful shutdown
CommandReceiver* receiver_ptr = nullptr;
void signalHandler(int signum) {
    if (receiver_ptr) {
        std::cout << "\nShutting down..." << std::endl;
        receiver_ptr->stop();
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <port>" << std::endl;
        return 1;
    }

    try {
        // Set up signal handler
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);

        // Create and run receiver
        CommandReceiver receiver(std::stoi(argv[1]));
        receiver_ptr = &receiver;
        receiver.run();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}