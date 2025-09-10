from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_launch')
    config_dir = os.path.join(pkg_share, 'config')
    return LaunchDescription([
        # 启动视频发布节点
        Node(
            package='camera_para_get',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            parameters=[os.path.join(config_dir, 'camera.yaml')],
            ),

        # 启动视频处理节点
        Node(
            package='solvepnp',
            executable='camera_info_subscriber',
            name='camera_info_subscriber',
            parameters=[
                os.path.join(config_dir, 'kf_para.yaml'),
                os.path.join(config_dir, 'timer.yaml')]
            )
            ])