import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtle_hunter'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items()
        )

    return LaunchDescription([
        SetParameter(name='delay', value=5.0),
        demo_nodes,
        Node(
            package='turtle_hunter',
            executable='fixed_frame',
            name='fixed_broadcaster',
        ),
    ])