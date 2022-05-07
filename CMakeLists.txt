cmake_minimum_required(VERSION 3.13)
project(bulletGame)

set(CMAKE_CXX_STANDARD 14)

add_executable(${PROJECT_NAME} main.cpp)

add_subdirectory(external/bullet)

target_include_directories(${PROJECT_NAME} PUBLIC external/bullet/include)

#target_link_directories(${PROJECT_NAME} PRIVATE external/glfw/src)

#target_link_libraries(${PROJECT_NAME} glfw)

