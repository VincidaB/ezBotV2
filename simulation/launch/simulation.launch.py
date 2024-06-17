# copyright option3 ensma
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():


    robot_package_name='ezbot-v2'
    simulation_package_name='ezbot-v2-simulation' 


    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    sim_time_arg =  DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(robot_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'verbose': 'true'}.items()
    )

    world = os.path.join(get_package_share_directory(simulation_package_name), 'worlds', 'table_with_panneaux.world')

    gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        #launch_arguments={'gz_args': ['-r -v4', world], 'on_exit_shutdown': 'true'}.items()
        launch_arguments={'gz_args': [' ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'EzBot',
                                   '-x','1.0',
                                   '-y','-1.0',
                                   '-z', '0.1' ],
                        output='screen')
    
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                   output='screen')


    delayed_spawner = TimerAction(
        period=5.0,
        actions=[spawn_entity],
    )

   
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )
    load_omnidirectional_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'omnidirectional_controller'],
        output='screen'
    )    
    load_omnidirectional_controller_other = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','omnidirectional_controller'],
        output='screen'
    )


    # Launch them all!
    return LaunchDescription([
        
        rsp,
        gz_server_cmd,
        spawn_entity,
        load_joint_state_broadcaster,
        load_omnidirectional_controller,
        sim_time_arg,
        bridge,
    ])