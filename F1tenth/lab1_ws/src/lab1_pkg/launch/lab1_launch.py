from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for parameters v and d
    declare_v = DeclareLaunchArgument('v', default_value='1.0', description='Speed parameter for talker')
    declare_d = DeclareLaunchArgument('d', default_value='0.5', description='Steering angle parameter for talker')

    # Define the talker node
    talker_node = Node(
        package='lab1_pkg',  # Replace with your package name
        executable='talker.py',  # Name of the talker node
        name='talker',
        output='screen',
        parameters=[{
            'v': LaunchConfiguration('v'),
            'd': LaunchConfiguration('d')
        }]
    )

    # Define the relay node
    relay_node = Node(
        package='lab1_pkg',  # Replace with your package name
        executable='relay.py',  # Name of the relay node
        name='relay',
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        declare_v,
        declare_d,
        talker_node,
        relay_node
    ])
