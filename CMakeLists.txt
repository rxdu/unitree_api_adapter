cmake_minimum_required(VERSION 3.8)
project(unitree_api_adapter)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

## Additional cmake module path
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")
list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

set(dependencies
    rclcpp
    controller_interface
    pluginlib
    unitree_go
    unitree_api
)

add_library(${PROJECT_NAME} SHARED
    src/unitree_api_adapter.cpp)
#target_link_libraries(${PROJECT_NAME} PUBLIC unitree_sdk2)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" PRIVATE src)
pluginlib_export_plugin_description_file(controller_interface unitree_api_adapter.xml)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

## The install and export configuration
# library targets that external packages may use
install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(some_dependency)

ament_package()
