from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动视频发布节点
        Node(
            package='camera_para_get',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            parameters=['/home/ypq/final_ws/src/my_launch/config/camera.yaml'],
            ),

        # 启动视频处理节点
        Node(
            package='solvepnp',
            executable='camera_info_subscriber',
            name='camera_info_subscriber',
            parameters=['/home/ypq/final_ws/src/my_launch/config/kf_para.yaml',
                        '/home/ypq/final_ws/src/my_launch/config/timer.yaml']
            )
            ])