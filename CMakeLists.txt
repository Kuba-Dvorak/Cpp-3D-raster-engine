cmake_minimum_required(VERSION 4.2)
project(Cpp_3D_engine_rasterizer)

set(CMAKE_CXX_STANDARD 20)
find_package(SDL2 REQUIRED)

add_executable(Cpp_3D_Engine main.cpp
        usefullLibs.cpp
        usefullLibs.h)

target_include_directories(Cpp_3D_Engine PUBLIC ${SDL2_INCLUDE_DIRS})
target_link_libraries(Cpp_3D_Engine ${SDL2_LIBRARIES})
