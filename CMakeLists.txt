cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

project(strange_attractors)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (viewer main.cpp)
target_compile_options(viewer
    PUBLIC
        -Wall
        -Wextra
        -O3
        -fsanitize=address,undefined,leak
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wunused
        -Woverloaded-virtual
        -pedantic-errors
        -Wnull-dereference
        -march=native
        -mtune=native
)

set_target_properties(viewer
    PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
)

target_link_libraries(viewer
    ${PCL_LIBRARIES}
    -fsanitize=address,undefined,leak
)
