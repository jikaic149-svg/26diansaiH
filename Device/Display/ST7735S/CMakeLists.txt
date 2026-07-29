add_library(device_st7735s STATIC
    "device_st7735s.c")

target_include_directories(device_st7735s PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_st7735s PUBLIC
    bsp
    mspm0_build_options)
