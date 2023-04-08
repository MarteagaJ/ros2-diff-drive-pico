cmake_minimum_required(VERSION 3.8)
project(diff_drive_pico)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  diff_drive_pico
  SHARED
  src/diff_drive_pico.cpp
)

find_package(lcm REQUIRED)
target_link_libraries(diff_drive_pico PUBLIC ${LCM_NAMESPACE}lcm)


target_compile_features(diff_drive_pico PUBLIC cxx_std_11)
target_include_directories(diff_drive_pico PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/diff_drive_pico>
)
ament_target_dependencies(
  diff_drive_pico PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFF_DRIVE_PICO_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diff_drive_pico.xml)

## EXPORTS
ament_export_targets(export_diff_drive_pico HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/diff_drive_pico
)
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf description/rviz
#   DESTINATION share/diff_drive_pico
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/diff_drive_pico
# )
install(TARGETS diff_drive_pico
  EXPORT export_diff_drive_pico
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
