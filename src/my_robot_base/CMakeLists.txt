cmake_minimum_required(VERSION 3.8)
project(my_robot_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖的功能包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(io_context REQUIRED)

# 添加可执行文件
add_executable(serial_comm_node src/serial_comm_node.cpp)

# 链接依赖库
ament_target_dependencies(serial_comm_node rclcpp std_msgs serial_driver sensor_msgs io_context)
# 安装可执行文件
install(TARGETS
  serial_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python脚本（规划/键盘），去除.py后缀便于 ros2 run
install(PROGRAMS scripts/angle_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME angle_planner)

install(PROGRAMS scripts/motion_mode_classification.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_mode_classification)

install(PROGRAMS scripts/control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control)

ament_package()