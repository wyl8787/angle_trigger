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
set(CMAKE_PROJECT_NAME cboard)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    applications/can/can_task.cpp
    applications/uart/uart_task.cpp
    applications/plotter/plotter_task.cpp
    applications/trigger/trigger_task.cpp

    
    sp_middleware/io/adc/adc.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    sp_middleware/io/buzzer/buzzer.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/io/dbus/dbus.cpp
    # sp_middleware/io/fdcan/fdcan.cpp
    sp_middleware/io/led/led.cpp
    sp_middleware/io/plotter/plotter.cpp
    # sp_middleware/io/servo/servo.cpp
    # sp_middleware/io/vision/vision.cpp
    # sp_middleware/io/ws2812/ws2812.cpp

    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/motor/dm_motor/dm_motor.cpp
    sp_middleware/motor/lk_motor/lk_motor.cpp
    sp_middleware/referee/pm02/pm02.cpp
    # sp_middleware/referee/vt02/vt02.cpp

    sp_middleware/tools/crc/crc.cpp
    # sp_middleware/tools/diff_drive/diff_drive.cpp
    # sp_middleware/tools/diff_gear/diff_gear.cpp
    # sp_middleware/tools/fuzzy_pid/fuzzy_pid.cpp
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
    # sp_middleware/tools/mecanum/mecanum.cpp
    # sp_middleware/tools/motor_composer/motor_composer.cpp
    sp_middleware/tools/pid/pid.cpp
    # sp_middleware/tools/swerve/swerve.cpp
    sp_middleware/tools/yaw_feedward/yaw_feedward.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    applications
    sp_middleware
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
