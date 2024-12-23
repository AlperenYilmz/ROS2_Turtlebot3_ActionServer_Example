import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtlebot_env = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle")

    controller_node = Node(
        package="turtlebot_final_project",
        executable="controller_node")

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    kill_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', 'kill', 'turtlesim/srv/Kill', '{"name": "turtle1"}']
    )

    empty_world_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("turtlebot3_gazebo"), "launch", "empty_world.launch.py")
        ))

    return LaunchDescription([turtlebot_env, controller_node, turtlesim_node, kill_turtle_cmd, empty_world_turtlebot3])