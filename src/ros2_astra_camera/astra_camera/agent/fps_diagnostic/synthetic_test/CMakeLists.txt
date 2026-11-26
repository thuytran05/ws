cmake_minimum_required(VERSION 3.8)
project(synthetic_image_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(image_pub main.cpp)
ament_target_dependencies(image_pub rclcpp sensor_msgs)
