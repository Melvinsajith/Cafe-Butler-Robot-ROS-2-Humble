import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Define hardcoded values that were present in the second code block, 
# but keep the original package name ('butler_pkg') for file path resolution.
PACKAGE_NAME = 'butler_pkg'      # <-- Name of your local ROS 2 package
ENTITY_NAME = 'restaurant_robot' # <-- New name for your spawned robot entity
WORLD_FILE_NAME = 'new_cafe.world' # <-- New hardcoded world file name


def launch_setup(context):
    # Arguments are resolved here, although we will primarily use the hardcoded paths/names below
    
    # --- 1. File Paths (Resolved) ---
    pkg_butler = get_package_share_directory(PACKAGE_NAME)
    
    # Paths for includes/nodes (using the resolved package path)
    rsp_launch_path = os.path.join(pkg_butler, 'launch', 'robot_pu.launch.py')
    gazebo_params_file = os.path.join(pkg_butler, 'config', 'gazebo_params.yaml')
    twist_mux_param_file = os.path.join(pkg_butler, 'config', "twist_mux.yaml")
    rviz_config_file = os.path.join(pkg_butler, 'config', 'main.rviz')
    world_path = os.path.join(pkg_butler, 'worlds', 'new_cafe.world')

    # --- 2. Included Launches ---
    
    # Include Robot State Publisher Launch (Calls robot_pu.launch.py)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_path]),
        # Argument is passed as a string for inclusion; conversion to bool happens in robot_pu.launch.py
        launch_arguments={'use_sim_time': 'true'}.items() 
    )

    # Include Gazebo launch (starts gzserver and gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path, # Pass the world file path
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # --- 3. ROS 2 Nodes (All use Python Boolean: True) ---
    
    # Spawn robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', ENTITY_NAME],
        output='screen',
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )

    # Twist mux node
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

    # RVIZ Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )

    # Controller Spawners
    controller_manager_pkg = 'controller_manager'
    
    diff_drive_spawner = Node(
        package=controller_manager_pkg,
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )

    joint_broad_spawner = Node(
        package=controller_manager_pkg,
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': True}] # Passed as Python Boolean
    )
    
    # Returning all nodes and included launches
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
    # Only need to declare the arguments required by the OpaqueFunction signature, 
    # even if we hardcode their values inside launch_setup.
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true', 
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='world_name',
            default_value='new_cafe_world_main.world',
            description='The name of the Gazebo world file to load'
        ),
        
        # Executes the launch_setup function
        OpaqueFunction(function=launch_setup),
    ])