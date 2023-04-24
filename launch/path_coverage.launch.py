# USAGE:

# . /usr/share/gazebo/setup.sh; source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; ros2 launch path_coverage path_coverage.launch.py


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_coverage',
            executable='path_coverage_node.py',
            name='path_coverage',
            output='screen',
            parameters=[{
                "boustrophedon_decomposition": True, # <!-- Whether to execute the boustrophedon decomposition --> default - "true"
                "border_drive": False, # <!-- Drive around the cell first --> default - "false"
                "robot_width": 0.15, # <!-- Width of each path --> default - "0.15"
                "costmap_max_non_lethal": 15, # <!-- Maximum costmap value to consider free --> default - "15"
                "base_frame": "base_link", # <!-- The robots base frame --> default - "base_link"
                "global_frame": "map"
            }]
        ),
    ])





