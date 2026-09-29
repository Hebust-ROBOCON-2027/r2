cmake_minimum_required(VERSION 3.8)
project(new_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 包含目录
include_directories(
  include
)

# 构建节点库 (包含 MoveToFixedPointAction 行为树节点)
add_library(${PROJECT_NAME}_node SHARED
  src/move_to_fixed_point_action.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav2_msgs
  std_msgs
)

# 构建独立的执行器可执行程序 new_bt_runner
add_executable(${PROJECT_NAME}_runner
  src/new_bt_runner.cpp
)

target_link_libraries(${PROJECT_NAME}_runner
  ${PROJECT_NAME}_node
)

ament_target_dependencies(${PROJECT_NAME}_runner
  rclcpp
  behaviortree_cpp
  ament_index_cpp
)

# 安装目标与资源
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_node)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  tf2_geometry_msgs
  nav2_msgs
)

ament_package()
