cmake_minimum_required(VERSION 3.5)
project(collision_ros2)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include/${PROJECT_NAME}/)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Build our plugin
add_library(model_push_ros2 SHARED src/ModelPushRos.cpp)
target_link_libraries(model_push_ros2 ${GAZEBO_LIBRARIES}) #[[<--- Nodig om Gazebo te gebruiken]]
target_link_libraries(model_push_ros2 Threads::Threads)
ament_target_dependencies(model_push_ros2 rclcpp) #[[<----- Nodig om ROS2 te gebruiken]]

add_library(collide SHARED src/CollidePluginRos.cpp)
target_link_libraries(collide ${GAZEBO_LIBRARIES})

 install(TARGETS collide model_push_ros2 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY world DESTINATION share/${PROJECT_NAME}/)
 
ament_package()
