import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
import pvporcupine
import sounddevice as sd
import numpy as np

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # Publisher for sending commands to PX4
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # Initialize Porcupine with your Access Key and custom wake word
        self.access_key = "T0X/oYqY5ha/S5io2KywIoP1r4xfzzULLBHYAQ8YWse3NN1q1Ox5UQ=="  # Replace with your Picovoice Access Key
        self.keyword_path = "/home/hn/Voice_command/Drone-Alpha_en_linux_v3_0_0.ppn"  # Path to your custom wake word file
        
        try:
            self.porcupine = pvporcupine.create(
                access_key=self.access_key,
                keyword_paths=[self.keyword_path]
            )
            self.get_logger().info("Porcupine initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Porcupine: {e}")
            raise
        
        # Initialize audio stream using sounddevice
        self.sample_rate = self.porcupine.sample_rate
        self.frame_length = self.porcupine.frame_length
        
        try:
            self.audio_stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype=np.int16,
                blocksize=self.frame_length,
                callback=self.audio_callback
            )
            self.get_logger().info("Audio stream initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize audio stream: {e}")
            raise

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Audio stream status: {status}")
        
        # Process the audio data with Porcupine
        pcm = indata.flatten().astype(np.int16)
        try:
            keyword_index = self.porcupine.process(pcm)
            if keyword_index >= 0:
                self.get_logger().info("Wake word detected! Arming drone...")
                self.arm_drone()
        except Exception as e:
            self.get_logger().error(f"Error processing audio: {e}")

    def arm_drone(self):
        # Create a VehicleCommand message to arm the drone
        msg = VehicleCommand()
        msg.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0   # 1 to arm, 0 to disarm
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1  # Set to the system ID of the drone (usually 1)
        msg.target_component = 1  # Set to the component ID (usually 1)
        msg.from_external = True
        self.publisher.publish(msg)
        self.get_logger().info("Arm command sent to PX4.")

    def start_listening(self):
        # Start the audio stream
        try:
            self.audio_stream.start()
            self.get_logger().info("Audio stream started. Listening...")
        except Exception as e:
            self.get_logger().error(f"Failed to start audio stream: {e}")

    def stop_listening(self):
        # Stop the audio stream
        try:
            self.audio_stream.stop()
            self.get_logger().info("Audio stream stopped.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop audio stream: {e}")

    def destroy_node(self):
        # Clean up resources
        self.stop_listening()
        if hasattr(self, 'porcupine'):
            self.porcupine.delete()
            self.get_logger().info("Porcupine resources released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    node.start_listening()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
