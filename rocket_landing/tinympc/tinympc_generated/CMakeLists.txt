cmake_minimum_required (VERSION 3.5)
project(TinyMPC VERSION 0.2.0 LANGUAGES CXX)

# compiler configuration
if (NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
    set (CMAKE_CXX_STANDARD_LIBRARIES "${CMAKE_CXX_STANDARD_LIBRARIES} -lm")
endif()

include_directories(include/Eigen)

add_subdirectory(src/tinympc)
add_subdirectory(examples)
