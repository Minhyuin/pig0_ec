cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMSISCORE "${CMAKE_SOURCE_DIR}/Drivers/CMSIS")

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME board_c)

add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32F407xx
    -DARM_MATH_CM4
    -DARM_MATH_MATRIX_CHECK
    -DARM_MATH_ROUNDING
    -DARM_MATH_LOOPUNROLL
    -DDISABLEFLOAT16) # need -D<macro> to define macro

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")
include("USERCODE/robot_core/drv_conf.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 启用输出log色块化
add_compile_options(-fdiagnostics-color=auto)

# 忽略编译警告,建议不要开启，因为有些警告可能是大错误，导致程序崩溃，so, 为了减少警告就要有好的代码规范
# add_compile_options(-w)

# 关闭指定警告（有效，但是有cuda文件时会报错）
add_compile_options(-Wno-unused-variable)
add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Wno-unused-function)
add_compile_options(-Wno-unused-value)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-missing-braces)
add_compile_options(-Wno-sequence-point)
add_compile_options(-Wno-pedantic)

# 禁用 'always_inline' 相关的警告
add_compile_options(-Wno-attributes)

# 禁用 '#pragma' 相关的警告 (要保证这些pragma能被gcc兼容)
add_compile_options(-Wno-unknown-pragmas)

# add_compile_options(-O0) add_compile_options(-Ofast)

# 添加.c文件
file(GLOB_RECURSE SOURCES "./USERCODE/robot_core/*.*")

# 忽略os keil port文件
file(GLOB_RECURSE REMOVE_SOURCES "./USERCODE/robot_core/OS/drv_freertos/portable/Keil/*.*")
list(REMOVE_ITEM SOURCES ${REMOVE_SOURCES})

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

# Add STM32CubeMX generated sources
add_subdirectory(./cmake/stm32cubemx)

if(USE_DSP)
    add_subdirectory(./lib/CMSIS-DSP/Source)
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user sources here
)

macro(FIND_INCLUDE_DIR result curdir)
    file(GLOB_RECURSE children "${curdir}/*.hpp" "${curdir}/*.h")
    set(dirlist "")

    foreach(child ${children})
        string(REGEX REPLACE "(.*)/.*" "\\1" LIB_NAME ${child})

        if(IS_DIRECTORY ${LIB_NAME})
            list(FIND dirlist ${LIB_NAME} list_index)

            if(${list_index} LESS 0)
                list(APPEND dirlist ${LIB_NAME})
            endif()
        endif()
    endforeach()

    set(${result} ${dirlist})
endmacro()

find_include_dir(INCLUDE_DIR_LIST ./USERCODE/robot_core)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${INCLUDE_DIR_LIST}

    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(USE_DSP)
    target_link_libraries(
        ${CMAKE_PROJECT_NAME} CMSISDSP # Add user defined libraries
    )
endif()
