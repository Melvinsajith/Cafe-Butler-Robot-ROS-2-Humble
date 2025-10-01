import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

# --- OPAQUE FUNCTION: This function performs all actions that require resolved LaunchConfiguration values ---
def launch_setup(context):
    # 1. Resolve arguments into strings using .perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    pkg_name = LaunchConfiguration('pkg_name').perform(context)
    xacro_file_name = LaunchConfiguration('xacro_file_name').perform(context)
    
    # 2. Get the full, resolved path to the XACRO file
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'description', xacro_file_name)

    # 3. Process the XACRO file into a simple URDF XML string
    # This step MUST happen after the LaunchConfigurations are resolved
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # 4. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config, # Pass the processed string
            'use_sim_time': use_sim_time
        }]
    )

    # 5. Joint State Publisher Node (Needed to manually control joints if GUI is desired)
    # Note: Use 'joint_state_publisher' instead of '_gui' if you want headless
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        # Other nodes like static_transform_publisher or controllers go here
    ]

# --- GENERATE LAUNCH DESCRIPTION: Declares arguments and calls the setup function ---
def generate_launch_description():
    
    # Arguments are only declared here, not resolved
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='pkg_name',
            default_value='butler_pkg', 
            description='Package name where description files are located'
        ),
        DeclareLaunchArgument(
            name='xacro_file_name',
            default_value='robot.urdf.xacro',
            description='The name of the xacro file to process'
        ),
        
        # OpaqueFunction executes the launch_setup function when arguments are ready
        OpaqueFunction(function=launch_setup)
    ])