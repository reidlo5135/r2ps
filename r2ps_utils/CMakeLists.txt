cmake_minimum_required(VERSION 3.8)
project(r2ps_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(SOURCES
  src/r2ps_utils/string.cpp
  src/r2ps_utils/process.cpp
  src/r2ps_utils/message.cpp
)

set(LIB_NAME r2ps_utils)

set(DEPENDS
  rclcpp
  std_msgs
  builtin_interfaces
)

add_library(${LIB_NAME} SHARED
  ${SOURCES}
)

ament_target_dependencies(${LIB_NAME}
  ${DEPENDS}
)

install(TARGETS ${LIB_NAME}
  EXPORT r2psUtilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${LIB_NAME})

ament_package()

install(EXPORT r2psUtilsTargets
  FILE r2psUtilsTargets.cmake
  NAMESPACE ${LIB_NAME}::
  DESTINATION lib/cmake/${LIB_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "r2psUtilsConfigVersion.cmake"
  VERSION 1.0
  COMPATIBILITY AnyNewerVersion
)

configure_file(r2psUtilsConfig.cmake.in r2psUtilsConfig.cmake @ONLY)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/r2psUtilsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/r2psUtilsConfigVersion.cmake"
  DESTINATION lib/cmake/r2ps_utils
)