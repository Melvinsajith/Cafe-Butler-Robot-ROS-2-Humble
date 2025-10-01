import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context):
    # 1. Resolve arguments to strings
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    
    # 2. File Paths (Correctly resolved using standard os.path.join with resolved strings)
    package_name = 'butler_pkg' 
    pkg_butler = get_package_share_directory(package_name)
    
    world_path = os.path.join(pkg_butler, 'worlds', world_name)
    rsp_launch_path = os.path.join(pkg_butler, 'launch', 'robot_pu.launch.py')

    gazebo_params_file = os.path.join(pkg_butler, 'config', 'gazebo_params.yaml')
    twist_mux_param_file = os.path.join(pkg_butler, 'config', "twist_mux.yaml")
    rviz_config_file = os.path.join(pkg_butler, 'config', 'main.rviz')

    # --- Actions ---
    
    # 3. Include Robot State Publisher Launch (Calls robot_pu.launch.py)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_path]),
        launch_arguments={'use_sim_time': use_sim_time}.items() 
    )

    # 4. Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # 5. Spawn Robot Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'butler_robot'],
        output='screen',
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )

    # 6. Twist Mux Node
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_param_file,
            {"use_sim_time": True} # Passed as Python Boolean
        ],
        remappings=[
            ("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")
        ]
    )

    # 7. RVIZ Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )

    # 8. Controller Spawners
    controller_manager_pkg = 'controller_manager'
    
    # Diff drive controller spawner
    diff_drive_spawner = Node(
        package=controller_manager_pkg,
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )

    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        package=controller_manager_pkg,
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )
    
    return [
        rsp,
        gazebo,
        spawn_entity,
        twist_mux,
        rviz_node,
        diff_drive_spawner,
        joint_broad_spawner,
    ]


def generate_launch_description():
    # --- Arguments Declarations ---
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true', # String default value passed to OpaqueFunction
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_name = DeclareLaunchArgument(
        name='world_name',
        default_value='cafe_world.world', # String default value
        description='The name of the Gazebo world file to load'
    )

    # --- LaunchDescription ---
    return LaunchDescription([
        declare_use_sim_time,
        declare_world_name,
        
        # OpaqueFunction executes the launch_setup function where all actions are run
        OpaqueFunction(function=launch_setup),
    ])