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
        BSP/Inc/fun.h
        BSP/Src/fun.c
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
        BSP/Src/test.c
        BSP/Inc/test.h
        BSP/Inc/hmi_tjc.h
        BSP/Src/hmi_tjc.c
        BSP/Src/lcd.c
        BSP/Inc/lcd.h)

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
        BSP/Src/fun.c
        BSP/Src/key_led.c
        BSP/Src/uart_dma.c
        BSP/Src/timer.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        BSP/Inc
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
