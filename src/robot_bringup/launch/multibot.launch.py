import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def create_spawn_entity(robot_name, x, y):
    return Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', f'/{robot_name}/robot_description',   # 🔧 use the namespaced topic
            '-entity', robot_name,
            '-x', str(x),
            '-y', str(y),
            '-robot_namespace', robot_name
        ],
        output='screen'
    )

def create_robot_state_publisher(robot_name, xacro_file):
    robot_description_config = xacro.process_file(xacro_file,  mappings={'name': robot_name})
    
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,  # Use the robot name as namespace
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'frame_prefix': robot_name + '/'  # Prefix for TF frames
        }]
    )

def generate_launch_description():
    package_name = 'robot_bringup'  # <-- Your package with rsp.launch.py

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_bringup'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # custom_world_path = os.path.join(pkg_path, 'worlds', 'soccer.world')
    
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    # launch_arguments={'world': custom_world_path}.items(),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # List of bots with names and spawn positions
    bots = [
        # ('bot1', 0.0, 2.0),
        ('bot2', 4.0, 0.0),
        ('bot3', -4.0, 0.0),
        ('bot4', 0.0, 4.0),
        ('bot5', 0.0, -4.0),
    ]

    # Create spawn and robot_state_publisher nodes for each bot
    spawn_entities = [create_spawn_entity(name, x, y) for name, x, y in bots]
    robot_state_publishers = [
        create_robot_state_publisher(name, xacro_file) for name, _, _ in bots
    ]
        # Launch them all!
    return LaunchDescription([
        *robot_state_publishers,
        gazebo,
        *spawn_entities
    ])


        #     # Spawn entity in Gazebo
        #     Node(
        #         package='gazebo_ros',
        #         executable='spawn_entity.py',
        #         arguments=[
        #             '-topic', 'robot_description',
        #             '-entity', name,
        #             '-x', str(x),
        #             '-y', str(y),
        #             '-z', '0.01'  # slight offset from ground
        #         ],
        #         output='screen'
        #     )
        # ])
    # Create a robot_state_publisher node
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_config.toxml(),
    #                  'frame_prefix': 'bot1' + '/'}]
    # )


