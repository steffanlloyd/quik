from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quik',
            executable='kinematics_service_node',
            parameters=[{os.path.join(get_package_share_directory('quik'), 'config', 'ik_service_kuka_kr6.yaml')}],
            output='screen'
        ),
    ])