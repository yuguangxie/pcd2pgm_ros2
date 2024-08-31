cmake_minimum_required(VERSION 3.5)
project(pcd2pgm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pcd2pgm src/pcd2pgm.cpp)

ament_target_dependencies(pcd2pgm rclcpp nav_msgs sensor_msgs PCL pcl_conversions)

# Link against the PCL libraries
target_link_libraries(pcd2pgm 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

install(TARGETS pcd2pgm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
