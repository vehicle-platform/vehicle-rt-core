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
set(CMAKE_PROJECT_NAME rt-core)

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

# T77: the generated Src/sysmem.c (a working newlib _sbrk, not editable: no USER CODE block) is
# dropped from the target and User/sbrk_fail.c (an _sbrk that always fails, invariant 5) is built
# instead. This survives a CubeMX regeneration, which rewrites only cmake/stm32cubemx.
get_target_property(RT_CORE_SOURCES ${CMAKE_PROJECT_NAME} SOURCES)
list(FILTER RT_CORE_SOURCES EXCLUDE REGEX "Src/sysmem\\.c$")
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY SOURCES ${RT_CORE_SOURCES})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    User/boot_checks.c
    User/sbrk_fail.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    User
    # Pure HAL-free headers shared with the host tests (hal/can_timing.h, hal/reset_cause.h)
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    # Generated bus constants (VEHICLE_CL250_BITRATE, PLATFORM_BITRATE); the submodule is pinned by the repo
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/vehicle-defs/gen/c/rt_core
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


# T77: fail if the linked image contains a heap (malloc/free/_sbrk). `cmake --build <dir> --target
# check_image_heap`; CI runs tools/check_image_heap.py on the same ELF.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_custom_target(check_image_heap
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_image_heap.py
                --nm ${TOOLCHAIN_PREFIX}nm $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Checking the image for malloc/_sbrk"
        VERBATIM)
endif()
