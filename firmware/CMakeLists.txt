# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
cmake_minimum_required(VERSION 3.13)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
include(pico_sdk_import.cmake)
project(luminas C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # for clangd

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

set(LUM_SRCS
    src/app/bringup_main.c
    # src/hal/i2c_bus.c
)

add_executable(luminas ${LUM_SRCS})

set_source_files_properties(${LUM_SRCS} PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Werror")


pico_set_program_name(luminas "luminas")
pico_set_program_version(luminas "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(luminas 1)
pico_enable_stdio_usb(luminas 0)

# Add the standard library to the build
target_link_libraries(luminas pico_stdlib hardware_i2c)

# Add the standard include files to the build
target_include_directories(luminas PRIVATE src)

pico_add_extra_outputs(luminas)

