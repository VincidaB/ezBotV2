# copyright option3 ensma
import os
import random

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


rand_half_range = 0.00

positions_yellow_list = [
    [-1.5+0.45/2, -1+0.45/2],
    [1.5-0.45/2, -0.225],
    [-1.17, 1-0.45/2],
]

def spawn_yellow_robot(context : LaunchContext, arg1 : LaunchConfiguration, arg2 : LaunchConfiguration, arg3 : LaunchConfiguration):
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    if context.perform_substitution(arg1) == 'True':
        i = random.randint(0, len(positions_yellow_list) - 1)
        x_pos = str(positions_yellow_list[i][0])
        y_pos = str(positions_yellow_list[i][1])
    else:
        x_pos = str(float(context.perform_substitution(arg2)) + random.uniform(-rand_half_range, rand_half_range))
        y_pos = str(float(context.perform_substitution(arg3)) + random.uniform(-rand_half_range, rand_half_range))
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot1/robot_description',
                                   '-entity', 'EzBot',
                                   '-x', x_pos,
                                   '-y', y_pos,
                                   '-z', '0.1',
                                    '--namespace', 'robot1' ,
                                    '--name', 'yellow_robot'],
                        output='screen')
    return [spawn_entity]


def generate_launch_description():

    robot_package_name='ezbot-v2'
    simulation_package_name='ezbot-v2-simulation' 

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    use_random_start = LaunchConfiguration('use_random_start', default=True)


    yellow_x = LaunchConfiguration('yellow_x', default=1.0)
    yellow_y = LaunchConfiguration('yellow_y', default=0)

    yellow_x_arg = DeclareLaunchArgument(
            'yellow_x',
            default_value=yellow_x,
            description='Position number from 0 to X for team yellow\'s robot'
    )

    yellow_y_arg = DeclareLaunchArgument(
            'yellow_y',
            default_value=yellow_y,
            description='Position number from 0 to X for team yellow\'srobot'
    )

    DeclareLaunchArgument(
            'blue_x',
            default_value='1.0',
            description='Position number from 0 to X for team blue\'s robot'
    )

    DeclareLaunchArgument(
            'blue_y',
            default_value='0',
            description='Position number from 0 to X for team blue\'srobot'
    )

    sim_time_arg =  DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
    )

    rsp1 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(robot_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'namespace' : 'robot1', 'color' : 'yellow'}.items()
    )
    
    #world = os.path.join(get_package_share_directory(simulation_package_name), 'worlds', 'table_with_everything.world')
    world = os.path.join(get_package_share_directory(simulation_package_name), 'worlds', 'Table_2025_poteaux.sdf')

    gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' ', ' ' ,world, ' ', '--record -r -v4 --seed 42 --gui-config ' , os.path.join(get_package_share_directory(simulation_package_name), 'config', 'gz_sim_new.config')], 'on_exit_shutdown': 'true'}.items()
    )
   
    load_joint_state_broadcaster1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','-c', '/robot1/controller_manager', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_omnidirectional_controller1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', '/robot1/controller_manager',
             '--set-state', 'active', 'omnidirectional_controller'],
        output='screen'
    )

    remote_calc_rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('remote_calc'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'namespace' : 'remote_calc1'}.items()
    )

    spawn_remote_calc = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'remote_calc1/robot_description',
                                    '-entity', 'remote_calc',
                                    '-x', '-0.25',
                                    '-y', '1.125',
                                    '-z', '0.75',
                                    '-R', '0.0',
                                    '-P', '0.0',
                                    '-Y', '1.57079632679',
                                    '--namespace', 'remote_calc1' ,
                                    '--name', 'remote_calc'],
                        output='screen')


    
    # To remap (rename) topics on the ROS 2 side, use the ':=' syntax after the ROS topic name.
    # Example: /gz_topic@ros_type@gz_type:=/ros2_topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory(simulation_package_name), 'config', 'gz_bridge.yaml')
        }],
        output='screen'
    )

    image_bridge = Node(package='ros_gz_image', executable='image_bridge',
                arguments=[
                    '/remote_calc1/remote_calc_cam1/img_raw',
                    '/robot1/camera1/img_raw',
                    '/robot1/camera2/img_raw',
                    '/robot1/camera3/img_raw',
                    ],
                output='screen')



    # Launch them all!
    return LaunchDescription([
        
        rsp1,
        gz_server_cmd,
        OpaqueFunction(function=spawn_yellow_robot, args=[use_random_start, yellow_x, yellow_y]),
        load_joint_state_broadcaster1,
        load_omnidirectional_controller1,
        sim_time_arg,
        yellow_x_arg,
        yellow_y_arg,
        remote_calc_rsp,
        spawn_remote_calc,
        bridge,
        image_bridge,
    ])