# Neobotix GmbH
# Author: Pradheep Padmanabhan

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

# MY_NEO_ROBOT = os.environ.get('MY_ROBOT', "mpo_700")
# MY_NEO_ENVIRONMENT = os.environ.get('MAP_NAME', "neo_workshop")

def generate_launch_description():
    use_multi_robots = LaunchConfiguration('use_multi_robots', default='False')
    use_amcl = LaunchConfiguration('use_amcl', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')

    # Declare launch arguments 'MY_ROBOT' and 'MAP_NAME' with default values and descriptions
    declare_my_robot_arg = DeclareLaunchArgument(
        'MY_ROBOT',
        default_value='mpo_700',
        description='Robot Types: "mpo_700", "mpo_500", "mp_400", "mp_500"') 
       
    declare_map_name_arg = DeclareLaunchArgument(
        'MAP_NAME',
        default_value='neo_workshop',
        description='Map Types: "neo_track1", "neo_track2", "neo_workshop"')
    
    # Create launch configuration variables for the robot and map name
    MY_NEO_ROBOT_ARG = LaunchConfiguration('MY_ROBOT')
    MY_NEO_ENVIRONMENT_ARG = LaunchConfiguration('MAP_NAME')
    
    # OpaqueFunction is used to perform setup actions during launch through a Python function
    def launch_setup(context, *args, **kwargs):
        MY_NEO_ROBOT = MY_NEO_ROBOT_ARG.perform(context)
        MY_NEO_ENVIRONMENT = MY_NEO_ENVIRONMENT_ARG.perform(context)

        map_dir = LaunchConfiguration(
            'map',
            default=os.path.join(
                get_package_share_directory('neo_simulation2'),
                'maps',
                MY_NEO_ENVIRONMENT+'.yaml'))

        param_file_name = 'navigation.yaml'
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('neo_simulation2'),
                'configs/'+MY_NEO_ROBOT,
                param_file_name))
        
        launch_actions = []
        nav2_launch_file_dir = os.path.join(get_package_share_directory('neo_nav2_bringup'), 'launch')

        # Include the localization_neo.launch.py file if use_amcl is not true
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_neo.launch.py']),
                condition=IfCondition(PythonExpression(['not ', use_amcl])),
                launch_arguments={
                    'map': map_dir,
                    'use_sim_time': use_sim_time,
                    'use_multi_robots': use_multi_robots,
                    'params_file': param_dir,
                    'namespace': namespace}.items(),
            )
        )

        # Include the localization_amcl.launch.py file if use_amcl is true
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_amcl.launch.py']),
                condition=IfCondition(use_amcl),
                launch_arguments={
                    'map': map_dir,
                    'use_sim_time': use_sim_time,
                    'use_multi_robots': use_multi_robots,
                    'params_file': param_dir,
                    'namespace': namespace}.items(),
            )
        )

        # Include the navigation_neo.launch.py file
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_neo.launch.py']),
                launch_arguments={'namespace': namespace,
                                'use_sim_time': use_sim_time,
                                'params_file': param_dir}.items()),
        )

        return launch_actions

    return LaunchDescription([
        declare_my_robot_arg,
        declare_map_name_arg,
        OpaqueFunction(function=launch_setup)
        ])
