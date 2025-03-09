#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <cmath>
#include <rclcpp/qos.hpp>

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

class UdpMavrosBridge : public rclcpp::Node {
public:
    UdpMavrosBridge(int port) : Node("udp_mavros_bridge"), udp_port_(port) {
        setupPublishers();
        setupSubscribers();
        setupClients();
        setupUdpSocket();
        RCLCPP_INFO(this->get_logger(), "UDP MAVROS Bridge listening on UDP port %d", udp_port_);
        
        // Set default yaw to 90 degrees (π/2 radians)
        current_yaw_ = M_PI / 2.0;
        
        position_timer_ = this->create_wall_timer(
            20ms, std::bind(&UdpMavrosBridge::publishPositionTarget, this));
        last_position_time_ = std::chrono::steady_clock::now();

        // // NEW: Create a timer for checking and maintaining offboard mode and arming
        // offboard_control_timer_ = this->create_wall_timer(
        //     1000ms, std::bind(&UdpMavrosBridge::offboardAndArmControl, this));

        startReceiveThread();
    }

    ~UdpMavrosBridge() {
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
    rclcpp::TimerBase::SharedPtr position_timer_;
    std::chrono::time_point<std::chrono::steady_clock> last_position_time_;
    int position_msg_count_ = 0;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_target_pub_;
    
    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;

    // Service clients
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    // Connection state
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;

    // // NEW timer for offboard/arming control
    // rclcpp::TimerBase::SharedPtr offboard_control_timer_;  

    int udp_port_;
    int sock_fd_;
    bool running_ = true;
    bool flag = true;
    std::thread receive_thread_;
    std::thread traj_thread_;
    float current_yaw_ = M_PI / 2.0;  // Default yaw set to 90 degrees

    void setupSubscribers() {
        // Configure QoS to be compatible with most mavros topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos,
            std::bind(&UdpMavrosBridge::stateCallback, this, std::placeholders::_1));
        
        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&UdpMavrosBridge::poseCallback, this, std::placeholders::_1));
    }
    
    void setupPublishers() {
        // Configure QoS to be compatible with mavros topics
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    
        local_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", qos);
        
        position_target_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", qos);
    }

    void setupClients() {
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
        
        // Wait for service availability
        while (!arming_client_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
        }
        
        while (!set_mode_client_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        
        while (!land_client_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for land service...");
        }
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        
        // Get roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        // Update current yaw
        current_yaw_ = yaw;
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

    

    // void traj_loop() {
    //     RCLCPP_INFO(this->get_logger(), "Trajectory thread started successfully");
    //     global_pos_z = -5;
    //     while (traj && rclcpp::ok()) {
    //         publishPositionTarget();
    //         std::this_thread::sleep_for(20ms);
    //     }
    //     RCLCPP_INFO(this->get_logger(), "Trajectory thread exiting");
    // }

    void traj_loop() {
        RCLCPP_INFO(this->get_logger(), "Trajectory thread started successfully");
        float target_altitude = -5.0;
        while (traj && rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Gradually decrease altitude until target is reached
                if (global_pos_z > target_altitude) {
                    global_pos_z -= 0.1;  // Adjust the step size as needed
                }
            }
            publishPositionTarget();
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(this->get_logger(), "Trajectory thread exiting");
    }
    
    // // NEW: Timer callback to check offboard mode and arming status
    // void offboardAndArmControl() {
    //     static auto last_request = this->now();
    //     // If not in offboard mode, try to set offboard every 5 seconds
    //     if (current_state_.mode != "OFFBOARD" && (this->now() - last_request).seconds() > 5.0) {
    //         setMode("OFFBOARD");
    //         RCLCPP_INFO(this->get_logger(), "Attempting to set OFFBOARD mode");
    //         last_request = this->now();
    //     }
    //     // If not armed, try to arm every 5 seconds
    //     else if (!current_state_.armed && (this->now() - last_request).seconds() > 5.0) {
    //         arm(true);
    //         RCLCPP_INFO(this->get_logger(), "Attempting to arm the vehicle");
    //         last_request = this->now();
    //     }
    // }

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

        // Switch to MANUAL mode
        if (udp_cmd.command == 400 && udp_cmd.param1 == 0) {
            traj = false;
            flag = true;    
            setMode("MANUAL");
            std::this_thread::sleep_for(1s);

            if (traj_thread_.joinable()) {
                traj_thread_.join();
            }
        }

        // Switch to OFFBOARD mode and start trajectory
        if (udp_cmd.command == 400 && udp_cmd.param1 == 1) {
            arm(true);
            // Start publishing setpoints immediately
            // Optionally, run a loop here for 2 seconds to publish a steady setpoint
            auto start_time = this->now();
            while ((this->now() - start_time).seconds() < 2.0) {
                publishSetpointPosition(0, 0, global_pos_z);  // or your desired initial setpoint
                std::this_thread::sleep_for(20ms);
            }
            // Now that offboard setpoints have been published for 2 seconds, switch to OFFBOARD mode.
            setMode("OFFBOARD");
            flag = false;
            // Wait a bit before starting the trajectory thread if needed
            std::this_thread::sleep_for(1s);
            traj = true;
            if (traj_thread_.joinable()) {
                traj_thread_.join();
            }
            traj_thread_ = std::thread(&UdpMavrosBridge::traj_loop, this);
        }
         
        // Takeoff sequence
        else if (udp_cmd.command == 176 && udp_cmd.param2 == 6.0f && flag == true) {
            std::this_thread::sleep_for(2s);
            arm(true);
            std::this_thread::sleep_for(1s);
            setMode("OFFBOARD");
            
            auto start_time = this->now();
            while ((this->now() - start_time).seconds() < 10) {
                publishSetpointPosition(0, 0, -3.0);  // Negative Z is up
                std::this_thread::sleep_for(20ms);
            }
            
            landDrone();
            RCLCPP_INFO(this->get_logger(), "Land command sent. Waiting for the drone to land...");
            std::this_thread::sleep_for(5s);
            arm(false);
            RCLCPP_INFO(this->get_logger(), "Disarm command sent.");
            std::this_thread::sleep_for(4s);
            setMode("MANUAL");
            RCLCPP_INFO(this->get_logger(), "Manual mode command sent.");
        }
       
        // Vertical movement (up/down)
        if (udp_cmd.command == 178 && udp_cmd.param2 == -1) {
            std::this_thread::sleep_for(2s);
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                global_pos_z -= .5;
            }

            RCLCPP_INFO(this->get_logger(),"global_pos_z: %.2f", global_pos_z);
        }
        if (udp_cmd.command == 178 && udp_cmd.param2 == 1) {
            std::this_thread::sleep_for(2s);
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                global_pos_z += .5;
            }

            RCLCPP_INFO(this->get_logger(),"global_pos_z: %.2f", global_pos_z);
        }
        
        // Forward movement (command 180)
        if (udp_cmd.command == 180) {
            std::this_thread::sleep_for(2s);
            
            float distance = udp_cmd.param1;  // Distance to move in meters
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Move forward in current yaw direction
                global_pos_x += distance * std::cos(current_yaw_);
                global_pos_y += distance * std::sin(current_yaw_);
            }
            
            RCLCPP_INFO(this->get_logger(), "Moving forward %.2f meters in yaw direction %.2f, new position: (%.2f, %.2f, %.2f)",
                      distance, current_yaw_, global_pos_x, global_pos_y, global_pos_z);
        }
        
        // Yaw control (command 179)
        if (udp_cmd.command == 179) {
            std::this_thread::sleep_for(2s);
            
            float yaw_direction = udp_cmd.param1;  // 1 for right, -1 for left
            float yaw_amount = udp_cmd.param2;     // Amount in radians (if specified, otherwise use default)
            
            if (yaw_amount == 0.0f) {
                yaw_amount = 1.57f;  // Default yaw amount (about 90 degrees)
            }
            
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (yaw_direction > 0) {  // Turn right
                    current_yaw_ += yaw_amount;
                    RCLCPP_INFO(this->get_logger(), "Turning right by %.2f radians, new yaw: %.2f", 
                               yaw_amount, current_yaw_);
                } else {  // Turn left
                    current_yaw_ -= yaw_amount;
                    RCLCPP_INFO(this->get_logger(), "Turning left by %.2f radians, new yaw: %.2f", 
                               yaw_amount, current_yaw_);
                }
                
                // Normalize yaw to [-π, π]
                while (current_yaw_ > M_PI) current_yaw_ -= 2 * M_PI;
                while (current_yaw_ < -M_PI) current_yaw_ += 2 * M_PI;
            }
        }
    }

    void arm(bool arm_cmd) {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm_cmd;
        
        auto future = arming_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Arming command sent: %s", arm_cmd ? "ARM" : "DISARM");
        
        // You could wait for the result if needed
        auto status = future.wait_for(5s);
        if (status == std::future_status::ready) {
            RCLCPP_INFO(get_logger(), "Arm result: %d", future.get()->success);
        }
    }

    void setMode(const std::string& mode) {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        
        auto future = set_mode_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Set mode command sent: %s", mode.c_str());
    }

    void landDrone() {
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = 0;
        request->latitude = 0;
        request->longitude = 0;
        request->min_pitch = 0;
        request->yaw = 0;
        
        auto future = land_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Land command sent");
    }

    void publishPositionTarget() {
        std::lock_guard<std::mutex> lock(mtx);
        
        if (traj) {
            auto target_msg = mavros_msgs::msg::PositionTarget();
            target_msg.header.stamp = this->get_clock()->now();
            target_msg.header.frame_id = "base_link";
            target_msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
            target_msg.type_mask = 
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::FORCE |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            
            // target_msg.position.x = global_pos_x;
            // target_msg.position.y = global_pos_y;
            // target_msg.position.z = global_pos_z;
            // target_msg.yaw = current_yaw_;

            // If your current global_pos_x, global_pos_y, global_pos_z are in ENU,
            // convert them to NED:
            // NED x = ENU y, NED y = ENU x, NED z = - (ENU z)
            float ned_x = global_pos_y;  
            float ned_y = global_pos_x;
            float ned_z = -global_pos_z;  // If global_pos_z is from ENU (up positive)

            // Option: if you want to command a specific altitude, override ned_z:
            // ned_z = 5.0; // in NED, 5 m down means -5 m altitude above ground

            target_msg.position.x = ned_x;
            target_msg.position.y = ned_y;
            target_msg.position.z = ned_z;
            target_msg.yaw = current_yaw_;
            
            position_target_pub_->publish(target_msg);
        }
        
        position_msg_count_++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_position_time_).count();

        if (elapsed > 1000) {  // Every second
            float rate = static_cast<float>(position_msg_count_) / (elapsed / 1000.0f);
            RCLCPP_INFO(this->get_logger(), "Position target publication rate: %.2f Hz", rate);
            if (rate < 2.0) {
                RCLCPP_WARN(this->get_logger(), "WARNING: Position target publication rate is below 2Hz");
            }
            position_msg_count_ = 0;
            last_position_time_ = now;
        }
    }

    void publishSetpointPosition(float pos_x, float pos_y, float pos_z) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = pos_x;
        pose.pose.position.y = pos_y;
        pose.pose.position.z = pos_z;
        
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        local_position_pub_->publish(pose);
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <udp_port>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpMavrosBridge>(std::stoi(argv[1]));
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
