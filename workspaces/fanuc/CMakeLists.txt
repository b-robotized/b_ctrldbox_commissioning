cmake_minimum_required(VERSION 3.8)
project(b_robotized_fanuc_demo)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
endif()

install(
  DIRECTORY urdf launch srdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
