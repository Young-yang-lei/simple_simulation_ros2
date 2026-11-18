cmake_minimum_required(VERSION 3.16)
project(cmake_utils)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
find_package(ament_cmake REQUIRED)

# Kept as an ament package for downstream compatibility. Prefer imported CMake targets.
install(DIRECTORY cmake cmake_modules DESTINATION share/${PROJECT_NAME})

ament_package()
