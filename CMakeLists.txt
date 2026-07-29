cmake_minimum_required(VERSION 3.16)
project(ros2_llm_safety_verifier LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROS-free verifier core (also buildable standalone via core/Makefile).
add_library(${PROJECT_NAME}_core core/src/verifier.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(verifier_node src/verifier_node.cpp)
target_link_libraries(verifier_node ${PROJECT_NAME}_core)
ament_target_dependencies(verifier_node rclcpp nav_msgs geometry_msgs std_msgs)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS verifier_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY core/include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
