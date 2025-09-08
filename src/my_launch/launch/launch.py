from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动视频发布节点
        Node(
            package='camera_para_get',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            parameters=['/home/ypq/final_ws/src/camera_para_get/config/camera.yaml'],
            ),

        # 启动视频处理节点
        Node(
            package='solvepnp',
            executable='camera_info_subscriber',
            name='camera_info_subscriber',
            parameters=[{'kf_process_noise': [1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4],
                         'kf_measurement_noise': [0.01,0.01,0.01,0.01]}]
            )
            ])