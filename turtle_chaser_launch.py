from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(package='turtle_chaser', executable='spawner', name='spawner'),
        Node(package='turtle_chaser', executable='chaser', name='chaser'),
    ])
