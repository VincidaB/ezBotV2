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

    rsp1 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(robot_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'namespace' : 'robot1'}.items()
    )
    rsp2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(robot_package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'namespace' : 'robot2'}.items()
    )
    
    world = os.path.join(get_package_share_directory(simulation_package_name), 'worlds', 'table_with_panneaux.world')

    gz_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        #launch_arguments={'gz_args': ['-r -v4', world], 'on_exit_shutdown': 'true'}.items()
        launch_arguments={'gz_args': [' ', world,' -v4 --gui-config ' , os.path.join(get_package_share_directory(simulation_package_name), 'config', 'gui.config.xml')], 'on_exit_shutdown': 'true'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity1 = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot1/robot_description',
                                   '-entity', 'EzBot',
                                   '-x','1.0',
                                   '-y','-1.0',
                                   '-z', '0.1',
                                    '--namespace', 'robot1' ,
                                    '--name', 'robot1entity'],
                        output='screen')
    spawn_entity2 = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot2/robot_description',
                                   '-entity', 'EzBot2',
                                   '-x','1.5',
                                   '-y','-1.0',
                                   '-z', '0.1' ,
                                    #'--namespace', 'robot2',
                                    '--name', 'robot2entity'],
                        output='screen')
    

   
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
    load_joint_state_broadcaster2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','-c', '/robot2/controller_manager', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_omnidirectional_controller2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '-c', '/robot2/controller_manager',
             '--set-state', 'active', 'omnidirectional_controller'],
        output='screen'
    )    

    
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/robot1/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                            '/robot1/omnidirectional_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist@gz.msgs.Twist'
                            ],
                output='screen')



    # Launch them all!
    return LaunchDescription([
        
        rsp1,
        rsp2,
        gz_server_cmd,
        spawn_entity1,
        spawn_entity2,
        load_joint_state_broadcaster1,
        load_omnidirectional_controller1,
        load_joint_state_broadcaster2,
        load_omnidirectional_controller2,
        sim_time_arg,
        bridge,
    ])