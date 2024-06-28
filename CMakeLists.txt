cmake_minimum_required(VERSION 3.8)
project(extract_data_from_bag)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(Boost REQUIRED
    COMPONENTS system filesystem thread date_time program_options)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

add_executable(extract_realsense_data_from_bag
    src/extract_realsense_data_from_bag.cc)
ament_target_dependencies(extract_realsense_data_from_bag
    rclcpp
    rcpputils
    rosbag2_cpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    image_transport
    pcl_conversions)
target_link_libraries(extract_realsense_data_from_bag
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable(extract_feynman_data_from_bag
    src/extract_feynman_data_from_bag.cc)
ament_target_dependencies(extract_feynman_data_from_bag
    rclcpp
    rcpputils
    rosbag2_cpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    image_transport
    pcl_conversions)
target_link_libraries(extract_feynman_data_from_bag
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES})

# Install nodes
install(
    TARGETS
        extract_realsense_data_from_bag
        extract_feynman_data_from_bag
    DESTINATION lib/${PROJECT_NAME})

ament_package()
