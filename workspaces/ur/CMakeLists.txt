cmake_minimum_required(VERSION 3.16)

project(b_robotized_ur_demo LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch config srdf
  DESTINATION share/b_robotized_ur_demo
)

ament_package()
