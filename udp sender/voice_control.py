import socket
import speech_recognition as sr
import struct
import time
from dataclasses import dataclass
from gpiozero import Button  # Import the Button class from gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory

# Define the GPIO pin for the button (adjust as needed)
BUTTON_PIN = 27

@dataclass
class VehicleCommand:
    command: int
    param1: float
    param2: float = 0.0
    target_system: int = 1
    target_component: int = 1
    source_system: int = 1
    source_component: int = 1
    from_external: bool = True

    def pack(self):
        packed_data = struct.pack(
            'IffBBBB?',
            self.command,
            self.param1,
            self.param2,
            self.target_system,
            self.target_component,
            self.source_system,
            self.source_component,
            self.from_external
        )

        return packed_data

class VoiceControlNode:
    def __init__(self, udp_ip, udp_port):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        
        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 400
        self.mic = sr.Microphone()
        
        # Command mappings
        self.command_handlers = {
            "alpha": self.arm_drone,
            "thank you": self.disarm_drone,
            "off board": self.switch_to_offboard_mode,
            "gripper open": self.open_gripper,
            "gripper close": self.close_gripper,
            "go up": self.throttle_up,
            "go down": self.throttle_down,
            "turn left": self.yaw_left,
            "turn right": self.yaw_right,
            "move forward": self.move_forward,
            "stop": self.stop_movement   # New stop command via voice
        }
        
        # Set up button using gpiozero
        factory = PiGPIOFactory()
        self.button = Button(BUTTON_PIN, pin_factory=factory)
        self.button.when_pressed = self.gpio_stop_callback  # Trigger callback on button press

    def gpio_stop_callback(self):
        # This callback is triggered when the button is pressed.
        print("GPIO button pressed, triggering stop command.")
        self.stop_movement()

    def send_command(self, cmd: VehicleCommand):
        try:
            sent = self.sock.sendto(cmd.pack(), (self.udp_ip, self.udp_port))
            print(f"Command sent via UDP: {cmd.command}")
        except Exception as e:
            print(f"Failed to send UDP message: {e}")

    def arm_drone(self):
        cmd = VehicleCommand(
            command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
            param1=1.0    # 1 to arm
        )
        self.send_command(cmd)
        print("Arm command sent")
        send_udp("ARMED")

    def disarm_drone(self):
        cmd = VehicleCommand(
            command=400,  # MAV_CMD_COMPONENT_ARM_DISARM
            param1=0.0    # 0 to disarm
        )
        self.send_command(cmd)
        print("Disarm command sent")
        send_udp("DISARM")

    def switch_to_offboard_mode(self):
        cmd = VehicleCommand(
            command=176,  # VEHICLE_CMD_DO_SET_MODE
            param1=1.0,
            param2=6.0    # OFFBOARD mode
        )
        self.send_command(cmd)
        print("Offboard mode command sent")
        send_udp("OFFBOARD")

    def open_gripper(self):
        cmd = VehicleCommand(
            command=187,  # Gripper command
            param1=1.0,
            param2=1.0
        )
        self.send_command(cmd)
        print("Gripper open command sent")
        send_udp("GRIPPER OPEN")

    def close_gripper(self):
        cmd = VehicleCommand(
            command=187,  # Gripper command
            param1=-1.0,
            param2=-1.0
        )
        self.send_command(cmd)
        print("Gripper close command sent")
        send_udp("GRIPPER CLOSE")

    def throttle_up(self):
        cmd = VehicleCommand(
            command=178,  # Custom throttle command
            param1=2,
            param2=-1
        )
        self.send_command(cmd)
        print("Throttle up command sent")
        send_udp("UP")
        
    def throttle_down(self):
        cmd = VehicleCommand(
            command=178,  # Custom throttle command
            param1=2,
            param2=1
        )
        self.send_command(cmd)
        print("Throttle down command sent")
        send_udp("DOWN")

    def yaw_left(self):
        cmd = VehicleCommand(
            command=179,  # Custom yaw command
            param1=-1.0
        )
        self.send_command(cmd)
        print("Yaw left command sent")
        send_udp("LEFT")

    def yaw_right(self):
        cmd = VehicleCommand(
            command=179,  # Custom yaw command
            param1=1.0
        )
        self.send_command(cmd)
        print("Yaw right command sent")
        send_udp("RIGHT")

    def move_forward(self):
        cmd = VehicleCommand(
            command=180,  # Custom forward movement command
            param1=10
        )
        self.send_command(cmd)
        print("Move forward command sent")
        send_udp("FORWARD")
        
    def stop_movement(self):
        # New stop command: command 181 stops forward movement (holds current position)
        cmd = VehicleCommand(
            command=181,  # Custom stop command
            param1=0.0
        )
        self.send_command(cmd)
        print("Stop command sent")
        send_udp("STOP")

    def run(self):
        print("Voice control running. Press Ctrl+C to exit.")
        # Adjust for ambient noise and listen for commands
        with self.mic as source:
            print("Adjusting for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            
            try:
                while True:
                    print("Listening...")
                    audio = self.recognizer.listen(source)
                    
                    try:
                        # Recognize speech using Google Web Speech API
                        text = self.recognizer.recognize_google(audio, language="en-EN")
                        print(f"Recognized: {text}")
                        
                        # Check for commands in the recognized text
                        text_lower = text.lower()
                        for command, handler in self.command_handlers.items():
                            if command in text_lower:
                                print(f"Command detected: {command}")
                                handler()
                                
                    except sr.UnknownValueError:
                        print("Could not understand audio")
                    except sr.RequestError as e:
                        print(f"Could not request results from Google Speech Recognition service; {e}")
            except KeyboardInterrupt:
                print("\nStopping voice control...")

    def cleanup(self):
        # Clean up resources if needed
        GPIO.cleanup()

def send_udp(message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message_bytes = message.encode('utf-8')
    ip_address = "127.0.0.1"
    port = 7880
    bytes_sent = sock.sendto(message_bytes, (ip_address, port))
    return bytes_sent
    
def main():
    import sys
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <udp_ip> <udp_port>")
        sys.exit(1)
        
    try:
        controller = VoiceControlNode(sys.argv[1], int(sys.argv[2]))
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
