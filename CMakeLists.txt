cmake_minimum_required(VERSION 3.20)
project(oled VERSION 1.0.0 LANGUAGES CXX)

# ── Compiler requirements ────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Dependencies ──────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "13.0")
        message(FATAL_ERROR "GCC >= 13 is required (found ${CMAKE_CXX_COMPILER_VERSION})")
    endif()
endif()

add_compile_options(
    -Wall -Wextra -Wpedantic
    -O2
    -march=native
)

# ── Library: oled ────────────────────────────────────────────────────────────
add_library(oled STATIC
    protocol/i2c.cpp
    src/oled_driver.cpp
)

target_link_libraries(oled PRIVATE 
    gpiod 
    Threads::Threads
)

target_include_directories(oled PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/protocol>
)

target_compile_features(oled PUBLIC cxx_std_20)

# ── Example executable ───────────────────────────────────────────────────────
add_subdirectory(example)