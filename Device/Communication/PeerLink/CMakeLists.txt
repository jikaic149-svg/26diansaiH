add_library(device_peer_link STATIC
    "device_peer_link.c")

target_include_directories(device_peer_link PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(device_peer_link PUBLIC
    algorithm_packet
    bsp
    mspm0_build_options)
