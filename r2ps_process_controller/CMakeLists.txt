cmake_minimum_required(VERSION 3.8)
project(r2ps_process_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include/
  include/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(r2ps_msgs REQUIRED)
find_package(r2ps_utils REQUIRED)

set(SOURCES
  src/process_controller.cpp
  src/ps_manager.cpp
  src/r2_manager.cpp
  src/main.cpp
)

set(DEPENDS
  rclcpp
  r2ps_msgs
  r2ps_utils
)

set(EXEC_NAME r2ps_process_controller)

add_executable(${EXEC_NAME}
  ${SOURCES}
)

ament_target_dependencies(${EXEC_NAME}
  ${DEPENDS}
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
