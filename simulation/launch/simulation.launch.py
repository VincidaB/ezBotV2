# copyright option3 ensma
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    robot_package_name='ezbot-v2'
    simulation_package_name='ezbot-v2-simulation' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(robot_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'EzBot',
                                   '-x','1.0',
                                   '-y','-1.0',
                                   '-z', '1.0' ],
                        output='screen')
    
    delayed_spawner = TimerAction(
        period=5.0,
        actions=[spawn_entity],
    )

    controller_params_file = os.path.join(get_package_share_directory(robot_package_name), 'config', 'omnidirectional_controller.yaml')
    
     
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        remappings = [('/controller_manager/robot_description', '/robot_description')],
    )

    delayed_controller_manager = TimerAction(
        period=1.0,
        actions=[controller_manager],
    )


    omnidrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omnidirectional_controller'],
    )
    delayed_omnidrive_spawner = TimerAction(
        period=10.0,
        actions=[omnidrive_spawner],
    )


    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            #default_value=[os.path.join('ezbot-v2-simulation','worlds','table2024.world'), ''],
            default_value=[os.path.join(get_package_share_directory("ezbot-v2-simulation"), 'worlds', 'table_with_panneaux.world'), ''],
            #default_value=[os.path.join(get_package_share_directory("ezbot-v2-simulation"), 'worlds', 'table2024Poteaux.world'), ''],
            description='SDF world file'),
        rsp,
        delayed_spawner,
        gazebo,
        delayed_controller_manager,
        delayed_omnidrive_spawner,
        #joystick
    ])