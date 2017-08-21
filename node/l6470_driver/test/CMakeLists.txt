#file (GLOB l6470_driver_src_files "*.h" "*.cpp")

add_executable(l6470_test
  l6470_test.cpp)

target_link_libraries(l6470_test
  l6470_driver)
