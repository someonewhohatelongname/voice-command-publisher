import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleCommand

class SimpleArmingNode(Node):
    def __init__(self):
        super().__init__('test_arm_node')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.timer = self.create_timer(1.0, self.arm_drone)

    def arm_drone(self):
        msg = VehicleCommand()
        msg.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0   # 1 to arm, 0 to disarm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1  
        msg.source_component = 1  
        msg.from_external = True
        self.publisher.publish(msg)
        self.get_logger().info("Arm command sent to PX4.")
        self.timer.cancel()  # Cancel the timer after sending the command

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArmingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

