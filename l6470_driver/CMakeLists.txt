set (l6470_driver_src_files GLOB *.cpp *.h)

add_library(l6470_driver ${l6470_driver_src_files})
target_link_libraries(l6470_driver mraa)
