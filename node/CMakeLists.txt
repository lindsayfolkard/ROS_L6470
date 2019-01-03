cmake_minimum_required(VERSION 3.5)

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
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
#force opensplice implementation
#find_package(rmw_opensplice_cpp REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(l6470_msgs REQUIRED)
find_package(l6470_srvs REQUIRED)

add_subdirectory(l6470_driver)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(l6470_node SHARED
  l6470_node.cpp)
target_compile_definitions(l6470_node
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(l6470_node
  "class_loader"
  "rclcpp"
  "l6470_msgs"
  "l6470_srvs"
  "l6470_driver")
rclcpp_register_node_plugins(l6470_node "l6470::L6470Node")
set(node_plugins "${node_plugins}l6470::L6470Node;$<TARGET_FILE:l6470_node>\n")

add_executable(l6470_main
  l6470_main.cpp)
target_link_libraries(l6470_main
  l6470_node
  l6470_driver)
ament_target_dependencies(l6470_main
  "rclcpp"
  "class_loader")

install(TARGETS
  l6470_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  l6470_main
  DESTINATION lib/${PROJECT_NAME})

#rclcpp_register_node_plugins(l6470_node "l6470::L6470Node")
#set(node_plugins "${node_plugins}l6470::L6470Node;$<TARGET_FILE:l6470_node>\n")

#add_executable(${PROJECT_NAME}_main main.cpp)
#target_link_libraries(${PROJECT_NAME}_main l6470_driver)
#ament_target_dependencies(${PROJECT_NAME}_main rclcpp example_interfaces ros_l6470_msgs)
#install(TARGETS ${PROJECT_NAME}_main
#        DESTINATION bin)

ament_package()
