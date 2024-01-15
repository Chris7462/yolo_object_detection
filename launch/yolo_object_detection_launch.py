from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = join(
        get_package_share_directory('yolo_object_detection'), 'params',
        'yolo_object_detection.yaml'
    )

    yolo_object_detection_node = Node(
        package='yolo_object_detection',
        executable='yolo_object_detection_node',
        name='yolo_object_detection_node',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        yolo_object_detection_node
    ])
