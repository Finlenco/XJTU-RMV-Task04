#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包路径
    pkg_share = get_package_share_directory('hik_camera_driver')
    
    # 声明启动参数
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='',
        description='相机IP地址'
    )
    
    camera_serial_arg = DeclareLaunchArgument(
        'camera_serial',
        default_value='',
        description='相机序列号'
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
    
    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time',
        default_value='1000.0',
        description='曝光时间(微秒)'
    )
    
    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='1.0',
        description='增益值'
    )
    
    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='bgr8',
        description='像素格式'
    )
    
    auto_reconnect_arg = DeclareLaunchArgument(
        'auto_reconnect',
        default_value='true',
        description='是否自动重连'
    )
    
    # 创建相机节点
    hik_camera_node = Node(
        package='hik_camera_driver',
        executable='hik_camera_node',
        name='hik_camera_driver',
        output='screen',
        parameters=[{
            'camera_ip': LaunchConfiguration('camera_ip'),
            'camera_serial': LaunchConfiguration('camera_serial'),
            'topic_name': LaunchConfiguration('topic_name'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'gain': LaunchConfiguration('gain'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            'reconnect_interval': 5
        }]
    )
    
    return LaunchDescription([
        camera_ip_arg,
        camera_serial_arg,
        topic_name_arg,
        frame_rate_arg,
        exposure_time_arg,
        gain_arg,
        pixel_format_arg,
        auto_reconnect_arg,
        hik_camera_node
    ])
