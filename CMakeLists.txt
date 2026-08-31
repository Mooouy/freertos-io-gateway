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
set(CMAKE_PROJECT_NAME freertos_io_gateway)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Native host unit tests build the pure modules with the host compiler and skip ARM setup.
option(BUILD_HOST_TESTS "Build native unit tests" OFF)
if(BUILD_HOST_TESTS)
    project(freertos_io_gateway_host C)
    enable_testing()
    add_subdirectory(tests/host)
    return()
endif()

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
    app/modbus/modbus_rtu.c
    # v0.3.0: RS485 总线唯一拥有者(Modbus 运行时)
    app/modbus/modbus_service.c
    # v0.3.0: BSM-0404 设备驱动(继续使用)
    app/devices/bsm0404.c
    # v0.4.0: 网关编排层(模型/规则/任务)
    app/gateway/gateway_model.c
    app/gateway/gateway_rules.c
    app/gateway/gateway_task.c
    # v0.5.0: ESP8266/MQTT 网络层(纯 AT 解析/编解码 + 非阻塞状态机任务)
    app/network/esp_at.c
    app/network/mqtt_codec.c
    app/network/esp8266_task.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    app/modbus
    app/devices
    app/gateway
    app/config
    app/network
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
