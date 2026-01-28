cmake_minimum_required(VERSION 3.8)
project(kdl_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

## INCLUDE(S)
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# executables
add_executable(kdl_example src/kdl_example.cpp)

# dependencies
ament_target_dependencies(kdl_example rclcpp std_msgs kdl_parser sensor_msgs)

target_link_libraries(kdl_example ${KDL_PARSER_LIBRARIES} Eigen3::Eigen)

install(TARGETS kdl_example
  DESTINATION lib/${PROJECT_NAME})


ament_package()
