add_library(device_ganwei STATIC
    "device_ganwei.c")

target_include_directories(device_ganwei PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_ganwei PUBLIC
    device_line_tracking
    mspm0_build_options)

target_link_libraries(device_ganwei PRIVATE
    bsp)
