import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('controlador_submarino')
    config_file = os.path.join(pkg_dir, 'launch', 'bridge_config.yaml')

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='puente_motores',
            output='screen',
            parameters=[{'config_file': config_file}],
        )
    ])
    
