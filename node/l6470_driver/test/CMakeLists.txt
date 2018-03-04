#file (GLOB l6470_driver_src_files "*.h" "*.cpp")

add_executable(stepper_motor_test
  stepper_motor_test.cpp)

add_executable(stop_motor
  stop_motor.cpp)

add_executable(config_unit_test
  config_unit_test.cpp)

add_executable(test_configs
  test_configs.cpp)

add_executable(command_unit_test
  command_unit_test.cpp)

target_link_libraries(stepper_motor_test
  l6470_driver)

target_link_libraries(test_configs
  l6470_driver)

target_link_libraries(stop_motor
  l6470_driver)

target_link_libraries(config_unit_test
   l6470_driver)

target_link_libraries(command_unit_test
   l6470_driver)
