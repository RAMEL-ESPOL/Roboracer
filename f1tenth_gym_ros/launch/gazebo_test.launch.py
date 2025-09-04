#!/usr/bin/env python3
# MIT License - F1Tenth Gym ROS with Gazebo support
# Based on original F1Tenth launch but adding Gazebo simulation

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    
    # Cargar configuración original de F1Tenth
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    
    # === PARTE NUEVA: GAZEBO ===
    package_name = 'f1tenth_gym_ros'

    # Parámetros de Gazebo
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Lanzar Gazebo Classic con mundo seleccionable
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': LaunchConfiguration('world'),  # ESTA LÍNEA FALTA
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to world file'
    )
    
    # === ROBOT STATE PUBLISHERS MODIFICADOS PARA GAZEBO ===
    
    # Ego robot con soporte Gazebo (usando nuestro nuevo XACRO)
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gazebo_racecar.xacro'), ' use_ros2_control:=false']),
            'use_sim_time': True  # Importante para Gazebo
        }],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    
    # Spawner del ego robot en Gazebo
    spawn_ego_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'ego_robot_description',
            '-entity', 'ego_racecar',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    # Robot oponente (si existe)
    if has_opp:
        opp_robot_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')]),
                'use_sim_time': True
            }],
            remappings=[('/robot_description', 'opp_robot_description')]
        )
        
        # Spawner del robot oponente
        spawn_opp_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'opp_robot_description',
                '-entity', 'opp_racecar',
                '-x', '2', '-y', '0', '-z', '0.1'  # Posición diferente
            ],
            output='screen'
        )
    
    # === PARTE ORIGINAL DE F1TENTH ===
    
    # Bridge node (comunicación con el simulador F1Tenth)
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {'use_sim_time': True}]  # Añadido use_sim_time
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')],
        parameters=[{'use_sim_time': True}]
    )
    
    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': True}
        ]
    )
    
    # Navigation lifecycle manager
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # Spawners de controladores ros2_control
    ackermann_controller_spawner = TimerAction(
        period=5.0,  # Esperar 5 segundos
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["ackermann_steering_controller"],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    joint_broadcaster_spawner = TimerAction(
        period=3.0,  # Esperar 3 segundos
        actions=[
            Node(
                package="controller_manager", 
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )
    
    # === CONSTRUIR LAUNCH DESCRIPTION ===
    
    # Gazebo primero
    ld.add_action(gazebo)
    
    # Robots
    ld.add_action(ego_robot_publisher)
    ld.add_action(spawn_ego_robot)
    
    if has_opp:
        ld.add_action(opp_robot_publisher)
        ld.add_action(spawn_opp_robot)
    
    # F1Tenth original
    ld.add_action(bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(world_arg)
    # ros2_control spawners
    ld.add_action(ackermann_controller_spawner)
    ld.add_action(joint_broadcaster_spawner)
    
    return ld