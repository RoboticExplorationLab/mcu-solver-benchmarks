cmake_minimum_required(VERSION 3.0.0)
list(APPEND CMAKE_MESSAGE_CONTEXT slap)
project(TinyMPC VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include/Eigen)

add_executable(quad_track src/admm.cpp src/quadrotor_track_traj.cpp)
target_include_directories (quad_track PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/lib/tinympc) # Include src/ directory instead of tinympc/
