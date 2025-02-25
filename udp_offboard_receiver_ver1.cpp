#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

struct UdpVehicleCommand {
    uint16_t command;
    float param1;
    float param2;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t source_system;
    uint8_t source_component;
    bool from_external;
};

bool traj = false;
std::mutex mtx;
float global_pos_x = 0;
float global_pos_y = 0;
float global_pos_z = 0;

class UdpRosBridge : public rclcpp::Node {
public:
    UdpRosBridge(int port) : Node("udp_ros_bridge"), udp_port_(port) {
        setupPublishers();
        setupUdpSocket();
        RCLCPP_INFO(this->get_logger(), "UDP ROS Bridge listening on UDP port %d", udp_port_);

        offboard_timer_ = this->create_wall_timer(
            20ms, std::bind(&UdpRosBridge::publishOffboardMode, this));
        last_offboard_time_ = std::chrono::steady_clock::now();

        startReceiveThread();
    }

    ~UdpRosBridge() {
        running_ = false;
        traj = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        if (traj_thread_.joinable()) {
            traj_thread_.join();
        }
        close(sock_fd_);
    }

private:
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    std::chrono::time_point<std::chrono::steady_clock> last_offboard_time_;
    int offboard_msg_count_ = 0;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;

    int udp_port_;
    int sock_fd_;
    bool running_ = true;
    bool flag = true;
    std::thread receive_thread_;
    std::thread traj_thread_;

    void setupPublishers() {
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    }

    void setupUdpSocket() {
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket");
        }

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(udp_port_);

        if (bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("Failed to bind UDP socket");
        }
    }

    void startReceiveThread() {
        receive_thread_ = std::thread([this]() {
            while (running_) {
                UdpVehicleCommand udp_cmd;
                struct sockaddr_in client_addr{};
                socklen_t client_len = sizeof(client_addr);

                ssize_t received = recvfrom(sock_fd_, &udp_cmd, sizeof(udp_cmd), 0,
                                              reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);

                if (received > 0) {
                    RCLCPP_INFO(this->get_logger(), "Received %zd bytes from %s:%d", 
                                received, inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                    processUdpCommand(udp_cmd);
                }
            }
        });
    }

    void traj_loop() {
        RCLCPP_INFO(this->get_logger(), "Trajectory thread started successfully");
        while (traj && rclcpp::ok()) {
            publishOffboardMode();
            publishTrajectorySetpoint(global_pos_x, global_pos_y, global_pos_z);
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(this->get_logger(), "Trajectory thread exiting");
    }

    void processUdpCommand(const UdpVehicleCommand& udp_cmd) {
        RCLCPP_INFO(this->get_logger(), 
            "UDP Command - command: %u, param1: %.2f, param2: %.2f, target_system: %u, target_component: %u, source_system: %u, source_component: %u, from_external: %d",
            udp_cmd.command,
            udp_cmd.param1,
            udp_cmd.param2,
            udp_cmd.target_system,
            udp_cmd.target_component,
            udp_cmd.source_system,
            udp_cmd.source_component,
            udp_cmd.from_external);

        auto msg = px4_msgs::msg::VehicleCommand();
        msg.param1 = udp_cmd.param1;
        msg.param2 = udp_cmd.param2;
        msg.command = udp_cmd.command;
        msg.target_system = udp_cmd.target_system;
        msg.target_component = udp_cmd.target_component;
        msg.source_system = udp_cmd.source_system;
        msg.source_component = udp_cmd.source_component;
        msg.from_external = udp_cmd.from_external;

        vehicle_command_pub_->publish(msg);

        if (udp_cmd.command == 400 && udp_cmd.param1 == 1) {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6.0);
            flag = false;
            std::this_thread::sleep_for(1s);
            traj = true;
            if (traj_thread_.joinable()) {
                traj_thread_.join();
            }
            traj_thread_ = std::thread(&UdpRosBridge::traj_loop, this);
        }else if (udp_cmd.command == 176 && udp_cmd.param2 == 6.0f && flag == true) { // Offboard mode
            std::this_thread::sleep_for(2s);
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            auto start_time = this->now();
            while ((this->now() - start_time).seconds() < 10) {
                publishOffboardMode();
                publishTrajectorySetpoint(0, 0, -3.0);
                std::this_thread::sleep_for(20ms);
            }
            landDrone();
            RCLCPP_INFO(this->get_logger(), "Land command sent. Waiting for the drone to land...");
            std::this_thread::sleep_for(5s);
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
            RCLCPP_INFO(this->get_logger(), "Disarm command sent.");
            std::this_thread::sleep_for(4s);
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 1);
            RCLCPP_INFO(this->get_logger(), "Manual mode command sent to PX4.");
        }
        if (udp_cmd.command == 400 && udp_cmd.param1 == 0) {
            traj = false;
            flag = true;
            if (traj_thread_.joinable()) {
                traj_thread_.join();
            }
        }
        
        if (udp_cmd.command == 178 && udp_cmd.param2 == -1) {
            std::this_thread::sleep_for(2s);
            auto start_time = this->now();

            {
                std::lock_guard<std::mutex> lock(mtx);
                global_pos_z -= .5;
            }

            RCLCPP_INFO(this->get_logger(),"global_pos_z: %.2f", global_pos_z);
        }
        if (udp_cmd.command == 178 && udp_cmd.param2 == 1) {
            std::this_thread::sleep_for(2s);
            auto start_time = this->now();

            {
                std::lock_guard<std::mutex> lock(mtx);
                global_pos_z += .5;
            }

            RCLCPP_INFO(this->get_logger(),"global_pos_z: %.2f", global_pos_z);
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Vehicle command sent: command=%d, param1=%.2f, param2=%.2f", 
                    command, param1, param2);
    }

    void landDrone() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }

    void publishOffboardMode() {
        //RCLCPP_INFO(this->get_logger(),"offboard_pub");
        auto mode_msg = px4_msgs::msg::OffboardControlMode();
        mode_msg.position = true;
        mode_msg.velocity = false;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(mode_msg);

        offboard_msg_count_++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_offboard_time_).count();

        if (elapsed > 1000) {  // Every second
            float rate = static_cast<float>(offboard_msg_count_) / (elapsed / 1000.0f);
            RCLCPP_INFO(this->get_logger(), "Offboard control mode publication rate: %.2f Hz", rate);
            if (rate < 2.0) {
                RCLCPP_WARN(this->get_logger(), "WARNING: Offboard control mode publication rate is below 2Hz");
            }
            offboard_msg_count_ = 0;
            last_offboard_time_ = now;
        }
    }

    void publishTrajectorySetpoint(float pos_x, float pos_y, float pos_z) {
        //RCLCPP_INFO(this->get_logger(),"trajectory_pub");
        auto traj_msg = px4_msgs::msg::TrajectorySetpoint();
        traj_msg.position = {pos_x, pos_y, pos_z};
        traj_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_pub_->publish(traj_msg);
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <udp_port>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpRosBridge>(std::stoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
