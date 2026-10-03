cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME gongxun_1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        BSP/Inc/headfile.h
        BSP/Src/key_led.c
        BSP/Inc/key_led.h
        BSP/Src/uart_dma.c
        BSP/Inc/uart_dma.h
        BSP/Src/timer.c
        BSP/Inc/timer.h
        BSP/Src/camera.c
        BSP/Inc/camera.h
        BSP/Src/lidar.c
        BSP/Inc/lidar.h
        Test/Src/basic_test.c
        Test/Inc/basic_test.h
        BSP/Inc/step_motor.h
        BSP/Src/step_motor.c
        BSP/Inc/Emm_V5.h
        BSP/Src/Emm_V5.c
        BSP/Inc/servo.h
        BSP/Src/servo.c
        BSP/Inc/ws2812.h
        BSP/Src/ws2812.c
        BSP/Inc/hwt101.h
        BSP/Src/hwt101.c
        BSP/Src/led_buzzer.c
        BSP/Inc/led_buzzer.h
        APP/Inc/chassis.h
        APP/Src/chassis.c
        APP/Inc/pid.h
        APP/Src/pid.c
        APP/Inc/catch.h
        APP/Src/catch.c
        APP/Inc/remind.h
        APP/Src/remind.c
        BSP/Src/hmi.c
        BSP/Inc/hmi.h
        APP/Src/hmi_app.c
        APP/Inc/hmi_app.h
        APP/Inc/bfs.h
        APP/Src/bfs.c
        Control/Inc/system.h
        Control/Src/system.c
        APP/Src/track_map.c
        APP/Inc/track_map.h
        APP/Src/obstacle_find.c
        APP/Inc/obstacle_find.h
        Test/Src/map_test.c
        Test/Inc/map_test.h
        Test/Src/map_stage_test.c
        Test/Inc/map_stage_test.h
        APP/Src/tf2d.c
        APP/Inc/tf2d.h
        Test/Src/cam_test.c
        Test/Inc/cam_test.h
        Test/Src/lidar_test.c
        Test/Inc/lidar_test.h
        Test/Src/run_test.c
        Test/Inc/run_test.h
        Test/Src/bfs_test.c
        Test/Inc/bfs_test.h
        Control/Src/task.c
        Control/Inc/task.h
        BSP/Src/debug_printf.c
        BSP/Inc/debug_printf.h
        Control/Inc/control.h
        Control/Src/control.c
        APP/Src/grid.c
        APP/Inc/grid.h
        APP/Src/map_data.c
        APP/Inc/map_data.h
        APP/Src/map_update.c
        APP/Inc/map_update.h
        APP/Src/map_plan.c
        APP/Inc/map_plan.h
        APP/Src/map_lidar.c
        APP/Inc/map_lidar.h

)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        BSP/Src/lidar.c
        BSP/Src/key_led.c
        BSP/Src/uart_dma.c
        BSP/Src/timer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        BSP/Inc
        APP/Inc
        Test/Inc
        Control/Inc
)

# Add include paths for STM32 HAL and USB middleware
list(APPEND CMAKE_INCLUDE_PATH
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_INCLUDE_PATH}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
