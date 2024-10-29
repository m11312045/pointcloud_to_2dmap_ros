from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_2dmap_ros2',
            executable='pointcloud_to_2dmap_node',
            name='pointcloud_to_2dmap_node',
            output='screen',
            parameters=[
                {"resolution": 0.1},
                {"map_width": 300},
                {"map_height": 300},
                {"min_points_in_pix": 1},
                {"max_points_in_pix": 2},
                {"min_height": 0.5},
                {"max_height": 3.0},
                {"dest_directory": "/home/user/ros2_ws/src/pointcloud_to_2dmap_ros/map/20241029_esoc_default/"},
                {"input_pcd": "/home/user/ros2_ws/src/pointcloud_to_2dmap_ros/map/20241029_esoc_default/map.pcd"},
                {"map_name": "2dmap"}
            ]
        )
    ])
