from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pkg_13224067'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='pkg_13224067',
            executable='autonomous_bridge.py',  # ini sementara, nanti kita benerin
            name='autonomous_bridge',
            output='screen',
            parameters=[config]
        )
    ])
