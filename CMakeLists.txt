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
set(CMAKE_PROJECT_NAME STM32G431CBU6_MMSD)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
message("CMake command: " ${CMAKE_COMMAND})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    DriverLayer/Src/drv_adc_sampling.c
    DriverLayer/Inc/drv_adc_sampling.h
    DriverLayer/Src/drv_spi_as5048a.c
    DriverLayer/Inc/drv_spi_as5048a.h
    DriverLayer/Src/drv_tim_pwm.c
    DriverLayer/Inc/drv_tim_pwm.h
    DriverLayer/Inc/drv_spi_as5048a_debug.h
    DriverLayer/Src/drv_spi_as5048a_debug.c
    DriverLayer/Inc/drv_nfault.h
    DriverLayer/Src/drv_nfault.c
    ControlLayer/Inc/ctl_math.h
    ControlLayer/Src/ctl_math.c
    ControlLayer/Inc/ctl_pid.h
    ControlLayer/Src/ctl_pid.c
    ControlLayer/Inc/ctl_foc_core.h
    ControlLayer/Src/ctl_foc_core.c
    ControlLayer/Inc/ctl_foc_debug.h
    ControlLayer/Src/ctl_foc_debug.c
    ControlLayer/Inc/ctl_foc_openloop.h
    ControlLayer/Src/ctl_foc_openloop.c
    ControlLayer/Inc/ctl_foc_current.h
    ControlLayer/Src/ctl_foc_current.c
    ControlLayer/Inc/ctl_foc_speed.h
    ControlLayer/Src/ctl_foc_speed.c
    ControlLayer/Inc/ctl_foc_position.h
    ControlLayer/Src/ctl_foc_position.c
    ControlLayer/Inc/ctl_foc_damper.h
    ControlLayer/Src/ctl_foc_damper.c
    ApplicationLayer/Inc/app_foc.h
    ApplicationLayer/Src/app_foc.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    DriverLayer/Inc
    ControlLayer/Inc
    ApplicationLayer/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Enable printf float support (newlib-nano disables %f by default)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
