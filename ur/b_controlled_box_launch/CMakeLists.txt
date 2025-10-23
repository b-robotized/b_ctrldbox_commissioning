cmake_minimum_required(VERSION 3.16)

project(b_controlled_box_launch LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/b_controlled_box_launch
)

ament_python_install_package(b_controlled_box_launch
  SCRIPTS_DESTINATION lib/b_controlled_box_launch
)
ament_package()
