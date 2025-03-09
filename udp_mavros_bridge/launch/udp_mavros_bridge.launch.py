# launch/udp_mavros_bridge.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the UDP port argument
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='14551',
        description='UDP port for the bridge to listen on'
    )

    # Create the node
    udp_mavros_bridge_node = Node(
        package='udp_mavros_bridge',
        executable='udp_mavros_bridge',
        name='udp_mavros_bridge',
        output='screen',
    #     parameters=[],
        arguments=[LaunchConfiguration('udp_port')]
    )

    # # Return the launch description
    # return LaunchDescription([
    #     udp_port_arg,
    #     udp_mavros_bridge_node
    # ])

    px4_config_path = os.path.join(
        get_package_share_directory('udp_mavros_bridge'),
        'config', 'px4_config.yaml'
    )
    px4_pluginlists_path = os.path.join(
        get_package_share_directory('udp_mavros_bridge'),
        'config', 'px4_pluginlists.yaml'
    )

    # Mavros Node
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
        {    
            # 'fcu_url': '/dev/ttyTHS1:1000000',
            'fcu_url': '/dev/ttyACM0:2000000',
            'gcs_url': 'udp://@10.42.0.1',
            #'gcs_url': 'udp://@10.100.18.240',
            'tgt_system': 1,
            'tgt_component': 1,
            'fcu_protocol': "v2.0",
            'respawn_mavros': "false",
            'namespace': "mavros",
        }],
    )

    return LaunchDescription([
        udp_port_arg,
        udp_mavros_bridge_node,
        mavros,
    ])