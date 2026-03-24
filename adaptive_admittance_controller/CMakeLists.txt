cmake_minimum_required(VERSION 3.16)
project(adaptive_admittance_controller)

# Basic compiler warnings and options
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_kdl
  tf2_eigen
  tf2_eigen_kdl
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_parser
  orocos_kdl
  pluginlib
  adaptive_admittance_controller_msgs
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)

# Generate parameter library
find_package(generate_parameter_library REQUIRED)

# Generate parameter library
generate_parameter_library(adaptive_admittance_controller_params
  config/adaptive_admittance_controller_params.yaml
)

# Create the controller as a shared library
add_library(adaptive_admittance_controller SHARED
  src/adaptive_admittance_controller.cpp
)

# Specify the C++ standard
target_compile_features(adaptive_admittance_controller PUBLIC cxx_std_17)

# Include directories for header files
target_include_directories(adaptive_admittance_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against the necessary dependencies and parameter library
ament_target_dependencies(adaptive_admittance_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(adaptive_admittance_controller PUBLIC adaptive_admittance_controller_params)

# Define the plugin for the controller
pluginlib_export_plugin_description_file(controller_interface adaptive_admittance_controller_plugin.xml)

# Install the shared library and headers
install(TARGETS
  adaptive_admittance_controller
  adaptive_admittance_controller_params
  EXPORT export_adaptive_admittance_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(export_adaptive_admittance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()