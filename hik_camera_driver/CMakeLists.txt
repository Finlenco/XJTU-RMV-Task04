cmake_minimum_required(VERSION 3.8)
project(hik_camera_driver)

# 设置ROS2路径
set(CMAKE_PREFIX_PATH "/opt/ros/humble" ${CMAKE_PREFIX_PATH})

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成compile_commands.json用于linter
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(include)
include_directories("${OpenCV_INCLUDE_DIRS}")
include_directories("/opt/ros/humble/include")

option(USE_MVS_SDK "Enable Hikvision MVS SDK support" ON)

if(USE_MVS_SDK)
  # 海康威视MVS SDK配置：优先 ENV MVS_HOME，其次 ~/opt/MVS，最后 /opt/MVS
  set(MVS_SDK_DIR "")
  if(DEFINED ENV{MVS_HOME} AND EXISTS "$ENV{MVS_HOME}")
    set(MVS_SDK_DIR "$ENV{MVS_HOME}")
  elseif(DEFINED ENV{HOME} AND EXISTS "$ENV{HOME}/opt/MVS")
    set(MVS_SDK_DIR "$ENV{HOME}/opt/MVS")
  elseif(EXISTS "/opt/MVS")
    set(MVS_SDK_DIR "/opt/MVS")
  endif()

  if(NOT MVS_SDK_DIR)
    message(FATAL_ERROR "MVS SDK not found. Please set MVS_HOME or install to ~/opt/MVS or /opt/MVS")
  endif()

  include_directories("${MVS_SDK_DIR}/include")

  # 添加MVS SDK库路径
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/64")
  else()
      set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/32")
  endif()

  link_directories("${MVS_LIB_DIR}")

  # RPATH 设置，运行时无需额外环境变量即可找到 libMvCameraControl.so
  set(CMAKE_BUILD_RPATH "${MVS_LIB_DIR}")
  set(CMAKE_INSTALL_RPATH "${MVS_LIB_DIR}")
  set(CMAKE_SKIP_BUILD_RPATH FALSE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  # 定义MVS SDK宏
  add_definitions(-DHAVE_MVS_SDK)

  message(STATUS "MVS SDK directory: ${MVS_SDK_DIR}")
  message(STATUS "MVS library directory: ${MVS_LIB_DIR}")
endif()

# 添加可执行文件
add_executable(hik_camera_node 
    src/hik_camera_node.cpp
    src/mvs_sdk_wrapper.cpp
)

add_executable(fps_monitor_node 
    src/fps_monitor_node.cpp
)

add_executable(param_info_node 
    src/param_info_node.cpp
)

# 链接依赖
ament_target_dependencies(hik_camera_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "rcl_interfaces"
)

ament_target_dependencies(fps_monitor_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

ament_target_dependencies(param_info_node
  "rclcpp"
  "std_msgs"
)

# 链接OpenCV
target_link_libraries(hik_camera_node ${OpenCV_LIBS})

# 链接MVS SDK库（可选）
if(USE_MVS_SDK)
  target_link_libraries(hik_camera_node 
      MvCameraControl
      pthread
  )
  message(STATUS "Linking MVS SDK libraries")
endif()

# 安装可执行文件
install(TARGETS hik_camera_node fps_monitor_node param_info_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
