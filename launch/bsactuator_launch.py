from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usbname',
            default_value='Arduino',
            description='Name of the USB device'
        ),
        DeclareLaunchArgument(
            'sudopass',
            default_value='',
            description='Sudo password for the device'
        ),
        DeclareLaunchArgument(
            'model',
            default_value='25mm01',
            description='Model of the Bambooshoot Actuator'
        ),
        Node(
            package='bsactuator_ros',  # ROS2 package name
            executable='bsactuator_ros',  # Python executable name as defined in setup.py (console_scripts)
            name='bsactuator_ros',
            output='screen',
            parameters=[{
                'usbname': LaunchConfiguration('usbname'),
                'sudopass': LaunchConfiguration('sudopass'),
                'model': LaunchConfiguration('model'),
            }]
        )
    ])
