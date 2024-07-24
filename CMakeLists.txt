cmake_minimum_required(VERSION 3.16)
project(uvms_ros2_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  casadi
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  udwadia_kalaba_controller_parameters
  src/udwadia_kalaba_controller_parameters.yaml
)

add_library(udwadia_kalaba_controller SHARED
  src/udwadia_controllers_base.cpp
  src/udwadia_kalaba_controller.cpp
  src/dynamics.cpp
)

target_link_libraries(udwadia_kalaba_controller PUBLIC casadi)

target_compile_features(udwadia_kalaba_controller PUBLIC cxx_std_17)
target_include_directories(udwadia_kalaba_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/udwadia_kalaba_controller>
)
target_link_libraries(udwadia_kalaba_controller PUBLIC
  udwadia_kalaba_controller_parameters
)
ament_target_dependencies(udwadia_kalaba_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(udwadia_kalaba_controller PRIVATE "UDWADIA_KALABA_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface uke_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  

endif()

# Collect all files within the casadi_lib directory and store them in a variable
file(GLOB CASADI_LIB_FILES "${CMAKE_CURRENT_SOURCE_DIR}/casadi_lib/*")

# INSTALL

# Install all files from the collected list to the destination directory
install(FILES ${CASADI_LIB_FILES} DESTINATION lib)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    ${PROJECT_NAME}
    udwadia_kalaba_controller_parameters
  EXPORT export_udwadia_kalaba_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_udwadia_kalaba_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
