from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone_2'),
        DeclareLaunchArgument('camera_topic', default_value='slot3'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data'),
        DeclareLaunchArgument('altimeter_topic', default_value='slot6/scan'),
        DeclareLaunchArgument('barometer_topic', default_value='air_pressure'),
        DeclareLaunchArgument('base_frame', default_value='drone_2'),
        DeclareLaunchArgument('show_matching', default_value='true'),
        Node(
            package='vinodom',
            executable='vinodom',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'namespace ': LaunchConfiguration('namespace')},
                        {'camera_topic': LaunchConfiguration('camera_topic')},
                        {'imu_topic': LaunchConfiguration('imu_topic')},
                        {'altimeter_topic': LaunchConfiguration('altimeter_topic')},
                        {'barometer_topic': LaunchConfiguration('barometer_topic')},
                        {'base_frame': LaunchConfiguration('base_frame')},
                        {'show_matching': LaunchConfiguration('show_matching')}],
            output='screen',
            emulate_tty=True
        )
    ])

