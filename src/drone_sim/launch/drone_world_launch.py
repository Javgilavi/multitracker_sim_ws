import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Path to the .sdf world file use for this simulator.
    world_path = os.path.join(get_package_share_directory('drone_sim'), 'worlds', 'world.sdf')

    # Path to the .sdf drone model file use for this simulator.
    drone_path = os.path.join(get_package_share_directory('drone_sim'), 'models', 'model_lidar.sdf')  # Change model for model_lidar to have drone with lidar

    # Path to the Gazebo launch file. This file launches Gazebo with ROS 2 integration.
    gazebo_launch_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([

        # Set the environment variable to make sure Gazebo can find its plugins. This path points to where ROS 2 Humble keeps the Gazebo plugins.
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib'),

        # Declare a launch argument to specify which world file to use for the simulation.
        DeclareLaunchArgument(
            'world',
            default_value=world_path, 
            description='Path to the SDF world file' 
        ),

        # Include the Gazebo launch description. This loads the Gazebo simulator and starts the simulation with the world file defined in the launch arguments.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),                  # Path to the Gazebo ROS launch file.
            launch_arguments={'world': LaunchConfiguration('world')}.items(),   # Pass the world argument to Gazebo.
        ),

        # Launch the drone model in the simulation. The 'spawn_entity.py' script is used to add a model (the drone) to the Gazebo simulation.
        Node(
            package='gazebo_ros',                           # This package provides the ROS interface to Gazebo.
            executable='spawn_entity.py',                   # Script to spawn entities (models) into Gazebo.
            arguments=['-entity', 'drone',                  # The name of the entity to spawn (the drone).
                       '-file', drone_path,                 # Path to the drone model's SDF file.
                       '-x', '0', '-y', '0', '-z', '3'],    # Initial position of the drone in the simulation (3 meters high on the Z axis).
            output='screen'                                 # Output to the terminal screen so you can see logs.
        ),

        # Launch a custom node to publish the position of the drone and the cubes.
        Node(
            package='drone_sim',        # The package where your custom node is located.
            executable='position_pub',  # The custom node that publishes the drone's position.
            name='position_pub',        # Name of the node as it will appear in ROS.
            output='screen'             # Output the node logs to the terminal screen.
        ),

    ])
