cmake_minimum_required(VERSION 2.8.3)

project(ros_l6470)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
#force opensplice implementation
#find_package(rmw_opensplice_cpp REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
#find_package(builtin_interfaces REQUIRED)
#find_package(rosidl_default_generators REQUIRED)


#ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(l6470_driver)
add_executable(${PROJECT_NAME}_main main.cpp)
target_link_libraries(${PROJECT_NAME}_main l6470_driver)
ament_target_dependencies(${PROJECT_NAME}_main rclcpp example_interfaces)# builtin_interfaces ${PROJECT_NAME}_msgs)
install(TARGETS ${PROJECT_NAME}_main
        DESTINATION bin)

ament_package()
