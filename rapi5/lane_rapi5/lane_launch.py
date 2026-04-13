from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            #namespace='turtlesim1',
            executable='pub',
            name='campub',
            #arguments=['--ros-args', '--log-level', 'info']
            output='screen',
        ),
        Node(
            package='dxl_nano',
            #namespace='turtlesim2',
            executable='sub',
            name='node_dxlsub',
            #emulate_tty=True,
            output='screen',
            #ros_arguments=['--log-level', 'warn']
        )
    ])
