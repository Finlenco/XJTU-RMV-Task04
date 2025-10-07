#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('hik_camera_driver')
    
    # 声明启动参数
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.1.100',
        description='相机IP地址'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/image_raw',
        description='图像话题名称'
    )
    
    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='相机帧率'
    )
    
    # 包含相机launch文件
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hik_camera.launch.py')
        ),
        launch_arguments={
            'camera_ip': LaunchConfiguration('camera_ip'),
            'topic_name': LaunchConfiguration('topic_name'),
            'frame_rate': LaunchConfiguration('frame_rate'),
        }.items()
    )
    
    # 启动RViz2
    rviz_config_file = os.path.join(pkg_share, 'config', 'camera_display.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        camera_ip_arg,
        topic_name_arg,
        frame_rate_arg,
        camera_launch,
        rviz_node
    ])
