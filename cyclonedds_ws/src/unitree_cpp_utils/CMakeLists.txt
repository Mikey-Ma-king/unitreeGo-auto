cmake_minimum_required(VERSION 3.10)
project(go2_cpp_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(pybind11_vendor)
find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

include_directories(
  include
  ${unitree_go_INCLUDE_DIRS}
)

pybind11_add_module(go2_cpp_utils SHARED
  src/main.cpp src/motor_crc.cpp
)
target_link_libraries(go2_cpp_utils PRIVATE
  rclcpp::rclcpp
  ${unitree_go_TARGETS}
  ${std_msgs_TARGETS}
)

# Sets PYTHON_INSTALL_DIR
_ament_cmake_python_get_python_install_dir()

install(
  TARGETS go2_cpp_utils
  DESTINATION "${PYTHON_INSTALL_DIR}"
)

ament_package()