# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path
import xacro

"""
Description:

This launch file is used to start a ROS2 simulation for a Neobotix robot in a specified environment. 
It sets up the Gazebo simulator with the chosen robot and environment, 
optionally starts the robot state publisher, and enables keyboard teleoperation.

You can launch this file using the following terminal commands:

1. `ros2 launch neo_simulation2 simulation.launch.py --show-args`
   This command shows the arguments that can be passed to the launch file.
2. `ros2 launch neo_simulation2 simulation.launch.py my_robot:=mpo_700 map_name:=neo_workshop use_sim_time:=true use_robot_state_pub:=true`
   This command launches the simulation with sample values for the arguments.

"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context: LaunchContext, my_neo_robot_arg, my_neo_env_arg, my_neo_robot_arm_arg, use_sim_time_arg, use_robot_state_pub_arg, use_gazebo_arg):
    # Create a list to hold all the nodes
    launch_actions = []
    # The perform method of a LaunchConfiguration is called to evaluate its value.
    my_neo_robot = my_neo_robot_arg.perform(context)
    my_neo_environment = my_neo_env_arg.perform(context)
    my_neo_robot_arm = my_neo_robot_arm_arg.perform(context)
    use_sim_time = use_sim_time_arg.perform(context).lower() == 'true'
    use_robot_state_pub = use_robot_state_pub_arg.perform(context)
    use_gazebo = use_gazebo_arg.perform(context)

    # Get the required paths for the world and robot robot_description_urdf
    default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', my_neo_environment + '.world')
    robot_description_urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+my_neo_robot+'/', my_neo_robot+'.urdf.xacro')
    xacro_args = {'use_gazebo': use_gazebo, 'arm': my_neo_robot_arm}  # Set the value of use_gazebo here
    # Use xacro to process the file
    robot_description_xacro = xacro.process_file(robot_description_urdf, mappings=xacro_args).toxml()

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items()
        )
    
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', my_neo_robot,'-topic', '/robot_description',], 
        output='screen')
    
    # if use_robot_state_pub is true, start the robot state publisher node
    if use_robot_state_pub == 'true':
        start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': robot_description_xacro}],
            )
        # Append the node to the launch_actions only if use_robot_state_pub is true
        launch_actions.append(start_robot_state_publisher_cmd)
    
    teleop = Node(
        package='teleop_twist_keyboard',
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e',
        name='teleop')

    # The required nodes can just be appended to the launch_actions list
    launch_actions.append(gazebo)
    launch_actions.append(spawn_entity)
    launch_actions.append(teleop)

    return launch_actions

def generate_launch_description():
    
    ld = LaunchDescription()

    # Declare launch arguments 'my_robot' and 'map_name' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
            'my_robot', default_value='mpo_700',
            description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"'
        ) 
       
    declare_map_name_arg = DeclareLaunchArgument(
            'map_name', default_value='neo_workshop',
            description='Map Types: "neo_track1", "neo_track2", "neo_workshop"'
        )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation clock if true'
        )
    
    declare_use_robot_state_pub_arg = DeclareLaunchArgument(
            'use_robot_state_pub', default_value='true',
            description='Use robot state publisher if true'
        )
    
    declare_use_gazebo_arg = DeclareLaunchArgument(
            'use_gazebo', default_value='true',
            description='Use gazebo if true'
        )
    
    declare_arm_cmd = DeclareLaunchArgument(
            'arm_name', default_value='',
            description='Arm Types:\n'
                'Elite Arms: ec66, cs66\n'
                'Universal Robotics: ur5, ur10, ur5e, ur10e'        
        )
    
    # Create launch configuration variables for the robot and map name
    my_neo_robot_arg = LaunchConfiguration('my_robot')
    my_neo_env_arg = LaunchConfiguration('map_name')
    my_neo_robot_arm_arg = LaunchConfiguration('arm_name')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    use_robot_state_pub_arg = LaunchConfiguration('use_robot_state_pub')
    use_gazebo_arg = LaunchConfiguration('use_gazebo')

    ld.add_action(declare_my_robot_arg)
    ld.add_action(declare_map_name_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_robot_state_pub_arg)
    ld.add_action(declare_use_gazebo_arg)
    ld.add_action(declare_arm_cmd)
    
    context_arguments = [my_neo_robot_arg, my_neo_env_arg, my_neo_robot_arm_arg, use_sim_time_arg, use_robot_state_pub_arg, use_gazebo_arg]
    opq_function = OpaqueFunction(function=launch_setup, 
                                  args=context_arguments)

    ld.add_action(opq_function)

    return ld