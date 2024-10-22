# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sq)

# Add the sources from the main src directory
FILE(GLOB app_sources src/*.c)

# Add the sources from the main src directory
target_sources(app PRIVATE ${app_sources})

# Add the sources from the main src directory
FILE(GLOB ui_sources src/ui/*.c)

# Add the sources from the main src directory
target_sources(app PRIVATE ${ui_sources})


# Add the 'ui' library from the src/ui directory
# add_subdirectory(src/ui)

# Ensure app can find the UI headers
# target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/ui)

# Add the sources from the main src directory
# target_sources(app PRIVATE ${app_sources})

# Link the ui library with the app
# target_link_libraries(app PRIVATE ui)

