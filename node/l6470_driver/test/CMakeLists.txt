#file (GLOB l6470_driver_src_files "*.h" "*.cpp")

add_executable(stepper_motor_test
  stepper_motor_test.cpp)

target_link_libraries(stepper_motor_test
  l6470_driver)
