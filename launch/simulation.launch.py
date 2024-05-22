# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
import os
from pathlib import Path


def generate_launch_description():
    
    # Declare launch arguments 'MY_ROBOT' and 'MAP_NAME' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
        'MY_ROBOT',
        default_value='mpo_700',
        description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"') 
       
    declare_map_name_arg = DeclareLaunchArgument(
        'MAP_NAME',
        default_value='neo_workshop',
        description='Map Types: "neo_track1", "neo_track2", "neo_workshop"')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='true',
        description='Use robot state publisher if true')
    
    # Create launch configuration variables for the robot and map name
    MY_NEO_ROBOT_ARG = LaunchConfiguration('MY_ROBOT')
    MY_NEO_ENVIRONMENT_ARG = LaunchConfiguration('MAP_NAME')

    USE_SIM_TIME_ARG = LaunchConfiguration('use_sim_time')
    USE_ROBOT_STATE_PUB_ARG = LaunchConfiguration('use_robot_state_pub')
    
    # OpaqueFunction is used to perform setup actions during launch through a Python function
    def launch_setup(context, *args, **kwargs):

        # The perform method of a LaunchConfiguration is called to evaluate its value.
        MY_NEO_ROBOT = MY_NEO_ROBOT_ARG.perform(context)
        MY_NEO_ENVIRONMENT = MY_NEO_ENVIRONMENT_ARG.perform(context)
        use_sim_time = USE_SIM_TIME_ARG.perform(context)
        use_robot_state_pub = USE_ROBOT_STATE_PUB_ARG.perform(context)

        default_world_path = os.path.join(get_package_share_directory('neo_simulation2'), 'worlds', MY_NEO_ENVIRONMENT + '.world')

        # robot_dir = LaunchConfiguration(
        #     'robot_dir',
        #     default=os.path.join(get_package_share_directory('neo_simulation2'),
        #         'robots/'+MY_NEO_ROBOT,
        #         MY_NEO_ROBOT+'.urdf'))
        
        gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
                ),
                launch_arguments={
                    'world': default_world_path,
                    'verbose': 'true',
                }.items()
            )
        urdf = os.path.join(get_package_share_directory('neo_simulation2'), 'robots/'+MY_NEO_ROBOT+'/', MY_NEO_ROBOT+'.urdf')
        
        spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',arguments=['-entity', MY_NEO_ROBOT, '-file', urdf], output='screen')
        
        # if use_robot_state_pub is true, start the robot state publisher node
        if use_robot_state_pub:
            start_robot_state_publisher_cmd = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[urdf])

        teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
            output='screen',
            prefix = 'xterm -e',
            name='teleop')

        return [gazebo, spawn_entity, start_robot_state_publisher_cmd, teleop]

    return LaunchDescription([
        declare_my_robot_arg,
        declare_map_name_arg,
        declare_use_sim_time_cmd,
        declare_use_robot_state_pub_cmd,
        OpaqueFunction(function=launch_setup)
        ])