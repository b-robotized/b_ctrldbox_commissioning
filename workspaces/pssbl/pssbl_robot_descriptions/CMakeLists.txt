cmake_minimum_required(VERSION 3.16)

project(pssbl_robot_descriptions LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config meshes urdf
  DESTINATION share/pssbl_robot_descriptions
)

ament_package()
