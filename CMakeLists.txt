cmake_minimum_required(VERSION 3.21)

project(f407_gateway VERSION 1.0.0 LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE MATCHES "^(Debug|Release)$")
    message(FATAL_ERROR "CMAKE_BUILD_TYPE must be Debug or Release")
endif()

set(CPU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VETX_FLASH.ld")

# 编译选项唯一真源（模式借自 TI-Template 的 build options INTERFACE 库）
add_library(f407_build_options INTERFACE)
target_compile_options(f407_build_options INTERFACE
    ${CPU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    $<$<CONFIG:Debug>:-Og;-g3;-gdwarf-4>
    $<$<CONFIG:Release>:-Os;-g1>
)
target_compile_definitions(f407_build_options INTERFACE
    STM32F407xx
    HSE_VALUE=25000000U
)

add_executable(${PROJECT_NAME}
    main.c
    startup/startup_stm32f407xx.s
    startup/system_stm32f4xx.c
    BSP/bsp_f407.c
    Device/BMI088/device_bmi088.c
    Device/ESP8266/device_esp8266.c
    Device/PeerLink/device_peer_link.c
    Device/QD4310/device_qd4310.c
    Device/VisionLink/device_vision_link.c
    APP/Algorithm/BallPositionControl/algorithm_ball_position_control.c
    APP/Algorithm/BallStateEstimator/algorithm_ball_state_estimator.c
    APP/Algorithm/ChassisAccelerationFusion/algorithm_chassis_acceleration_fusion.c
    APP/Algorithm/ChassisImuAcceleration/algorithm_chassis_imu_acceleration.c
    APP/Algorithm/LevelCurrentControl/algorithm_level_current_control.c
    APP/Algorithm/Packet/src/algorithm_packet.c
    APP/Algorithm/RodAngleMap/algorithm_rod_angle_map.c
    APP/Algorithm/SPSC_Ring/src/algorithm_spsc_ring.c
    APP/Gateway/app_gateway.c
    Task/task_app.c
    FreeRTOS/Kernel/tasks.c
    FreeRTOS/Kernel/queue.c
    FreeRTOS/Kernel/list.c
    FreeRTOS/Kernel/portable/GCC/ARM_CM4F/port.c
    FreeRTOS/Config/freertos_support.c
)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

target_include_directories(${PROJECT_NAME} PRIVATE
    .
    config
    BSP
    Device/BMI088
    Device/ESP8266
    Device/PeerLink
    Device/QD4310
    Device/VisionLink
    APP/Algorithm/BallPositionControl
    APP/Algorithm/BallStateEstimator
    APP/Algorithm/ChassisAccelerationFusion
    APP/Algorithm/ChassisImuAcceleration
    APP/Algorithm/LevelCurrentControl
    APP/Algorithm/Packet/include
    APP/Algorithm/RodAngleMap
    APP/Algorithm/SPSC_Ring/include
    APP/Gateway
    Task
    FreeRTOS/Config
    FreeRTOS/Kernel/include
    FreeRTOS/Kernel/portable/GCC/ARM_CM4F
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
)

target_link_libraries(${PROJECT_NAME} PRIVATE f407_build_options m c)

target_link_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
    LINK_DEPENDS "${LINKER_SCRIPT}")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating HEX/BIN and reporting image size"
)
