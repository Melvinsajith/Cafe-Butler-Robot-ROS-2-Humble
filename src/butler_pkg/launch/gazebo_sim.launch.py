import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# IMPORTANT: Replace 'my_cafe.world' with the exact name of your world file!
WORLD_FILE_NAME = '/home/melvin/butler_ws/src/butler_pkg/worlds/new_cafe_world_main' 

def generate_launch_description():

    # 1. Get the path to your world file
    # This assumes your world file is placed inside a 'worlds' folder in your package.
    # E.g., ~/butler_ws/src/butler_pkg/worlds/my_cafe.world
    pkg_butler = get_package_share_directory('butler_pkg')
    world_path = os.path.join(pkg_butler, 'worlds', WORLD_FILE_NAME)
    
    # 2. Get the Gazebo launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # 3. Start the main Gazebo launch file, passing the custom world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # CRUCIAL: Pass your world file path to the 'world' argument
        launch_arguments={'world': world_path}.items(),
    )

    # Return the LaunchDescription, only starting Gazebo
    return LaunchDescription([
        gazebo,
    ])