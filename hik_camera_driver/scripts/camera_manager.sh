#!/bin/bash

# 海康威视相机系统管理脚本 🚀
# 功能: 启动、测试、安装依赖、清理系统

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 默认参数（置空=不覆盖 YAML）。仅显式传参时才覆盖
CAMERA_IP=""
CAMERA_SERIAL=""
TOPIC_NAME=""
FRAME_RATE=""
EXPOSURE_TIME=""
GAIN=""
PIXEL_FORMAT=""
AUTO_RECONNECT=""
RECONNECT_INTERVAL=""
USE_RVIZ=""
MONITOR_FPS=""

# 显示帮助信息
show_help() {
    echo -e "${BLUE}海康威视相机系统管理脚本${NC}"
    echo ""
    echo "用法: $0 [命令] [选项]"
    echo ""
    echo "命令:"
    echo "  start     启动完整相机系统"
    echo "  test      测试系统功能"
    echo "  install   安装依赖"
    echo "  clean     清理构建文件"
    echo "  status    查看系统状态"
    echo "  help      显示帮助信息"
    echo ""
    echo "选项:"
    echo "  -i, --ip IP地址              相机IP地址"
    echo "  -s, --serial 序列号           相机序列号"
    echo "  -t, --topic 话题名            图像话题名称"
    echo "  -f, --fps 帧率                设定帧率"
    echo "  -e, --exposure 曝光时间       曝光时间(微秒)"
    echo "  -g, --gain 增益               增益值"
    echo "  -p, --pixel 像素格式          像素格式"
    echo "  --no-rviz                     不启动RViz"
    echo "  --no-monitor                  不启动帧率监控"
    echo ""
    echo "示例:"
    echo "  $0 start -i 192.168.1.100 -f 60.0"
    echo "  $0 test --no-rviz"
    echo "  $0 clean"
}

# 检查ROS2环境
check_ros2_env() {
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}❌ ROS2环境未设置，正在加载...${NC}"
        source /opt/ros/humble/setup.bash
    fi
    
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}❌ 无法加载ROS2环境，请检查安装${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ ROS2环境: $ROS_DISTRO${NC}"
}

# 检查工作空间
check_workspace() {
    if [ ! -f "install/setup.bash" ]; then
        echo -e "${YELLOW}⚠️  工作空间未构建，正在编译...${NC}"
        colcon build --packages-select hik_camera_driver
        if [ $? -ne 0 ]; then
            echo -e "${RED}❌ 编译失败${NC}"
            exit 1
        fi
    fi
    
    source install/setup.bash
    echo -e "${GREEN}✅ 工作空间已加载${NC}"
}

# 启动系统
start_system() {
    echo -e "${BLUE}🚀 启动海康威视相机系统...${NC}"
    
    # 检查环境
    check_ros2_env
    check_workspace
    
    # 构建启动参数（仅显式传参时才覆盖 YAML）
    # 方案：将覆盖项写入临时YAML，通过 override_params 传入 launch 文件
    LAUNCH_ARGS_ARR=()
    
    # 生成临时覆盖参数文件
    TMP_DIR="/tmp/hik_camera_driver"
    mkdir -p "$TMP_DIR"
    OVERRIDE_FILE="$TMP_DIR/override_$(date +%s).yaml"
    
    {
        echo "hik_camera_driver:"
        echo "  ros__parameters:"
        [ -n "$CAMERA_IP" ] && echo "    camera_ip: \"$CAMERA_IP\""
        [ -n "$CAMERA_SERIAL" ] && echo "    camera_serial: \"$CAMERA_SERIAL\""
        [ -n "$TOPIC_NAME" ] && echo "    topic_name: \"$TOPIC_NAME\""
        [ -n "$FRAME_RATE" ] && echo "    frame_rate: $FRAME_RATE"
        [ -n "$EXPOSURE_TIME" ] && echo "    exposure_time: $EXPOSURE_TIME"
        [ -n "$GAIN" ] && echo "    gain: $GAIN"
        [ -n "$PIXEL_FORMAT" ] && echo "    pixel_format: \"$PIXEL_FORMAT\""
        [ -n "$AUTO_RECONNECT" ] && echo "    auto_reconnect: $AUTO_RECONNECT"
        [ -n "$RECONNECT_INTERVAL" ] && echo "    reconnect_interval: $RECONNECT_INTERVAL"
    } > "$OVERRIDE_FILE"
    
    # 如果覆盖文件除了头部外没有有效条目，则不传入
    if grep -qE "^\s{4}\w" "$OVERRIDE_FILE"; then
        LAUNCH_ARGS_ARR+=("override_params:=$OVERRIDE_FILE")
    else
        rm -f "$OVERRIDE_FILE"
    fi
    
    # launch 级别开关
    if [ -n "$USE_RVIZ" ]; then LAUNCH_ARGS_ARR+=("use_rviz:=$USE_RVIZ"); fi
    if [ -n "$MONITOR_FPS" ]; then LAUNCH_ARGS_ARR+=("monitor_fps:=$MONITOR_FPS"); fi
    
    echo -e "${CYAN}📋 启动参数(空=使用YAML默认):${NC}"
    echo "  相机IP: ${CAMERA_IP:-<YAML>}"
    echo "  相机序列号: ${CAMERA_SERIAL:-<YAML>}"
    echo "  话题名称: ${TOPIC_NAME:-<YAML>}"
    echo "  帧率: ${FRAME_RATE:-<YAML>}"
    echo "  曝光时间: ${EXPOSURE_TIME:-<YAML>}"
    echo "  增益: ${GAIN:-<YAML>}"
    echo "  像素格式: ${PIXEL_FORMAT:-<YAML>}"
    echo "  自动重连: ${AUTO_RECONNECT:-<YAML>}"
    echo "  启动RViz: ${USE_RVIZ:-<YAML>}"
    echo "  帧率监控: ${MONITOR_FPS:-<YAML>}"
    echo ""
    
    # 启动系统
    echo -e "${BLUE}🎬 启动系统...${NC}"
    ros2 launch hik_camera_driver hik_camera_system.launch.py "${LAUNCH_ARGS_ARR[@]}"
}

# 测试系统
test_system() {
    echo -e "${BLUE} 测试海康威视相机系统...${NC}"
    
    # 检查环境
    check_ros2_env
    check_workspace
    
    # 测试1: 编译测试
    echo -e "${YELLOW} 测试1: 编译测试...${NC}"
    colcon build --packages-select hik_camera_driver
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}编译成功${NC}"
    else
        echo -e "${RED}❌ 编译失败${NC}"
        return 1
    fi
    
    # 测试2: 节点启动测试
    echo -e "${YELLOW} 测试2: 节点启动测试...${NC}"
    ros2 run hik_camera_driver hik_camera_node --ros-args -p topic_name:="/test_image" &
    NODE_PID=$!
    sleep 3
    
    if ps -p $NODE_PID > /dev/null; then
        echo -e "${GREEN}相机节点启动成功${NC}"
        kill $NODE_PID 2>/dev/null
    else
        echo -e "${RED}❌ 相机节点启动失败${NC}"
        return 1
    fi
    
    # 测试3: 帧率监控测试
    echo -e "${YELLOW}📋 测试3: 帧率监控测试...${NC}"
    ros2 run hik_camera_driver fps_monitor_node --ros-args -p monitor_topic:="/test_image" &
    MONITOR_PID=$!
    sleep 2
    
    if ps -p $MONITOR_PID > /dev/null; then
        echo -e "${GREEN}帧率监控节点启动成功${NC}"
        kill $MONITOR_PID 2>/dev/null
    else
        echo -e "${RED}❌ 帧率监控节点启动失败${NC}"
        return 1
    fi
    
    # 测试4: 参数信息测试
    echo -e "${YELLOW}📋 测试4: 参数信息测试...${NC}"
    ros2 run hik_camera_driver param_info_node --ros-args -p target_node_name:="hik_camera_driver" &
    PARAM_PID=$!
    sleep 2
    
    if ps -p $PARAM_PID > /dev/null; then
        echo -e "${GREEN}参数信息节点启动成功${NC}"
        kill $PARAM_PID 2>/dev/null
    else
        echo -e "${RED}❌ 参数信息节点启动失败${NC}"
        return 1
    fi
    
    # 测试5: 参数测试
    echo -e "${YELLOW}📋 测试5: 参数测试...${NC}"
    ros2 run hik_camera_driver hik_camera_node --ros-args -p frame_rate:=15.0 -p exposure_time:=2000.0 &
    NODE_PID=$!
    sleep 3
    
    if ps -p $NODE_PID > /dev/null; then
        echo -e "${GREEN}参数设置成功${NC}"
        kill $NODE_PID 2>/dev/null
    else
        echo -e "${RED}❌ 参数设置失败${NC}"
        return 1
    fi
    
    echo -e "${GREEN}所有测试通过！${NC}"
}

# 安装依赖
install_dependencies() {
    echo -e "${BLUE}安装系统依赖...${NC}"
    
    # 更新包列表
    echo -e "${YELLOW}更新包列表...${NC}"
    sudo apt update
    
    # 安装ROS2依赖
    echo -e "${YELLOW}安装ROS2依赖...${NC}"
    sudo apt install -y \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-rviz2 \
        ros-humble-sensor-msgs \
        ros-humble-std-msgs
    
    # 安装OpenCV
    echo -e "${YELLOW}安装OpenCV...${NC}"
    sudo apt install -y \
        libopencv-dev \
        python3-opencv
    
    # 安装编译工具
    echo -e "${YELLOW}安装编译工具...${NC}"
    sudo apt install -y \
        build-essential \
        cmake \
        git
    
    echo -e "${GREEN}依赖安装完成${NC}"
}

# 清理系统
clean_system() {
    echo -e "${BLUE}清理构建文件...${NC}"
    
    # 停止所有相关进程
    echo -e "${YELLOW}停止相关进程...${NC}"
    pkill -f "hik_camera_driver" 2>/dev/null
    pkill -f "fps_monitor" 2>/dev/null
    pkill -f "param_info" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    
    # 清理构建目录
    echo -e "${YELLOW}清理构建目录...${NC}"
    rm -rf build/ install/ log/
    
    echo -e "${GREEN}清理完成${NC}"
}

# 查看系统状态
show_status() {
    echo -e "${BLUE}系统状态检查...${NC}"
    
    # 检查ROS2环境
    if [ -n "$ROS_DISTRO" ]; then
        echo -e "${GREEN}ROS2环境: $ROS_DISTRO${NC}"
    else
        echo -e "${RED}❌ ROS2环境未设置${NC}"
    fi
    
    # 检查工作空间
    if [ -f "install/setup.bash" ]; then
        echo -e "${GREEN}工作空间已构建${NC}"
    else
        echo -e "${YELLOW}⚠️  工作空间未构建${NC}"
    fi
    
    # 检查运行中的节点
    echo -e "${CYAN}📋 运行中的节点:${NC}"
    ros2 node list 2>/dev/null | grep -E "(hik_camera|fps_monitor|param_info)" || echo "  无相关节点运行"
    
    # 检查话题
    echo -e "${CYAN}📋 相关话题:${NC}"
    ros2 topic list 2>/dev/null | grep -E "(image_raw|camera_fps|param_info)" || echo "  无相关话题"
}

# 解析命令行参数
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            start)
                COMMAND="start"
                shift
                ;;
            test)
                COMMAND="test"
                shift
                ;;
            install)
                COMMAND="install"
                shift
                ;;
            clean)
                COMMAND="clean"
                shift
                ;;
            status)
                COMMAND="status"
                shift
                ;;
            help|--help|-h)
                show_help
                exit 0
                ;;
            -i|--ip)
                CAMERA_IP="$2"
                shift 2
                ;;
            -s|--serial)
                CAMERA_SERIAL="$2"
                shift 2
                ;;
            -t|--topic)
                TOPIC_NAME="$2"
                shift 2
                ;;
            -f|--fps)
                FRAME_RATE="$2"
                shift 2
                ;;
            -e|--exposure)
                EXPOSURE_TIME="$2"
                shift 2
                ;;
            -g|--gain)
                GAIN="$2"
                shift 2
                ;;
            -p|--pixel)
                PIXEL_FORMAT="$2"
                shift 2
                ;;
            --no-rviz)
                USE_RVIZ=false
                shift
                ;;
            --no-monitor)
                MONITOR_FPS=false
                shift
                ;;
            *)
                echo -e "${RED}❌ 未知参数: $1${NC}"
                show_help
                exit 1
                ;;
        esac
    done
}

# 主函数
main() {
    # 解析参数
    parse_args "$@"
    
    # 检查命令
    if [ -z "$COMMAND" ]; then
        echo -e "${RED}❌ 请指定命令${NC}"
        show_help
        exit 1
    fi
    
    # 执行命令
    case $COMMAND in
        start)
            start_system
            ;;
        test)
            test_system
            ;;
        install)
            install_dependencies
            ;;
        clean)
            clean_system
            ;;
        status)
            show_status
            ;;
        *)
            echo -e "${RED}❌ 未知命令: $COMMAND${NC}"
            show_help
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"
