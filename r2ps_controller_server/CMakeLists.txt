cmake_minimum_required(VERSION 3.8)
project(r2ps_controller_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(FILES 
  package.json
  DESTINATION share/${PROJECT_NAME}/
)
 
install(DIRECTORY 
  config
  dist
  launch
  node_modules
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
) 
