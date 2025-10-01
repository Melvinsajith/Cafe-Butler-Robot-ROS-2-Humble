import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

# --- OPAQUE FUNCTION: Executes logic when arguments are resolved ---
def launch_setup(context):
    # 1. Resolve arguments into strings and Boolean
    # .perform(context) resolves the LaunchConfiguration into a string.
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    # Convert resolved string ('true' or 'false') to a Python Boolean (True/False)
    use_sim_time_bool = use_sim_time_str in ['true', '1', 't', 'y', 'yes'] 
    
    pkg_name = LaunchConfiguration('pkg_name').perform(context)
    xacro_file_name = LaunchConfiguration('xacro_file_name').perform(context)
    
    # 2. Get the full, resolved path to the XACRO file
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'description', xacro_file_name)

    # 3. Process the XACRO file into a simple URDF XML string
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # 4. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time_bool              # Correctly passed as Boolean
        }]
    )

    # 5. Joint State Publisher Node (Needed to visualize joints or debug)
    # Note: Using 'joint_state_publisher' instead of '_gui' is often better for simulation
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time_bool}], # Correctly passed as Boolean
        output='screen'
    )
    
    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
    ]

# --- GENERATE LAUNCH DESCRIPTION: Declares arguments and calls the setup function ---
def generate_launch_description():
    
    # Arguments are only declared here
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