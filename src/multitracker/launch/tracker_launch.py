import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the config file for rviz2
    rviz_config_dir = os.path.join(get_package_share_directory('multitracker'), 'config', 'my_config.rviz')

    return LaunchDescription([

        # Launch the node for the tracker itself
        Node(
            package='multitracker',        # The package where your custom node is located.
            executable='tracker',  # The custom node that publishes the drone's position.
            name='tracker',        # Name of the node as it will appear in ROS.
            output='screen'             # Output the node logs to the terminal screen.
        ),

        # Launch rviz2 with our custom configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]  # Cargar el archivo de configuración .rviz
        )
    ])
