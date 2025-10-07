#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
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
        default_value='Vir81799215',
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
        description='设定帧率'
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
    
    # 已移除测试图像开关，统一使用真实相机帧
    
    auto_reconnect_arg = DeclareLaunchArgument(
        'auto_reconnect',
        default_value='true',
        description='是否自动重连'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz2'
    )
    
    monitor_fps_arg = DeclareLaunchArgument(
        'monitor_fps',
        default_value='true',
        description='是否启动帧率监控'
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

    # 静态TF：world -> camera_link（单位变换，避免RViz Fixed Frame 缺失）
    static_tf_world_to_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_link'],
        output='screen'
    )

    # 可选：camera_link -> camera_optical_frame（符合REP-103的相机光学坐标系）
    static_tf_cam_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_link_to_optical',
        arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'camera_link', 'camera_optical_frame'],
        output='screen'
    )
    
    # 创建帧率监控节点
    fps_monitor_node = Node(
        package='hik_camera_driver',
        executable='fps_monitor_node',
        name='fps_monitor',
        output='screen',
        parameters=[{
            'monitor_topic': LaunchConfiguration('topic_name'),
            'update_interval': 1.0
        }],
        condition=IfCondition(LaunchConfiguration('monitor_fps'))
    )
    
    # 启动RViz2
    rviz_config_file = os.path.join(pkg_share, 'config', 'camera_display.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # 创建参数信息显示节点
    param_info_node = Node(
        package='hik_camera_driver',
        executable='param_info_node',
        name='param_info',
        output='screen',
        parameters=[{
            'camera_ip': LaunchConfiguration('camera_ip'),
            'camera_serial': LaunchConfiguration('camera_serial'),
            'topic_name': LaunchConfiguration('topic_name'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'exposure_time': LaunchConfiguration('exposure_time'),
            'gain': LaunchConfiguration('gain'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect')
        }]
    )
    
    return LaunchDescription([
        # 声明参数
        camera_ip_arg,
        camera_serial_arg,
        topic_name_arg,
        frame_rate_arg,
        exposure_time_arg,
        gain_arg,
        pixel_format_arg,
        auto_reconnect_arg,
        use_rviz_arg,
        monitor_fps_arg,
        
        # 启动节点
        hik_camera_node,
        static_tf_world_to_cam,
        static_tf_cam_to_optical,
        
        # 延迟启动其他节点，确保相机节点先启动
        TimerAction(
            period=2.0,
            actions=[
                fps_monitor_node,
                param_info_node,
                rviz_node
            ]
        )
    ])
