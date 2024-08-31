from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd2pgm',
            executable='pcd2pgm',
            name='pcd2pgm',
            output='screen',
            parameters=[
                {'file_directory': '/home/yunle/autodrive/maps/'},
                {'file_name': 'AreaBC'},
                {'thre_z_min': 0.2},
                {'thre_z_max': 2.0},
                {'flag_pass_through': 0},
                {'thre_radius': 0.5},
                {'map_resolution': 0.05},
                {'thres_point_count': 10},
                {'map_topic_name': 'map'}
            ]
        )
    ])
