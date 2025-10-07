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
    
    # 声明启动参数（仅保留功能性开关）
    
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
    
    # 参数文件（默认）
    # 使用安装后的共享目录中的参数文件（推荐生产模式）
    params_file = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    # 创建相机节点
    hik_camera_node = Node(
        package='hik_camera_driver',
        executable='hik_camera_node',
        name='hik_camera_driver',
        output='screen',
        parameters=[params_file]
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
        parameters=[{'monitor_topic': '/image_raw', 'update_interval': 1.0}],
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
        parameters=[params_file]
    )
    
    return LaunchDescription([
        # 声明参数
        # 参数由 YAML 提供为默认值，支持通过 --ros-args -p 覆盖
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
