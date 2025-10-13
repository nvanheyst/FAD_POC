import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# New import for the parameter fix
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Get the current directory where the launch file is run
    current_dir = os.path.abspath('.') 

    # Path to your custom world file
    world_file = os.path.join(current_dir, 'camera_setup_room.sdf')

    # Path to your URDF file
    urdf_file = os.path.join(current_dir, 'robot.urdf.xacro')

    # 1. Launch Gazebo Simulator
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    # 2. Process the URDF file
    robot_description = Command(['xacro ', urdf_file])

    # 3. Start the Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            # WRAP the robot_description parameter here
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }]
    )

    # 4. Spawn the robot into Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', 
                   '-name', 'dingo_o', 
                   '-allow_renaming', 'true'],
        output='screen'
    )
    
    # 5. Spawn the ROS2 controllers
    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_base_controller", "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        gz_sim_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        controller_spawner_node
    ])