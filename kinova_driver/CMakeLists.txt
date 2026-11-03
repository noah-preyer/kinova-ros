cmake_minimum_required(VERSION 3.8)
project(kinova_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kinova_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

###########
## Build ##
###########

# Wrappers around the Kinova vendor API. No ROS 1 dependencies.
add_library(kinova_driver_core SHARED
  src/fake_kinova_device.cpp
  src/kinova_api.cpp
  src/kinova_comm.cpp
  src/kinova_ros_types.cpp
)
target_include_directories(kinova_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kinova_driver_core PUBLIC
  rclcpp
  kinova_msgs
  geometry_msgs
  tf2
)
target_link_libraries(kinova_driver_core PUBLIC dl)

# ros2_control hardware interface plugin (kinova_driver/KinovaSystem).
add_library(kinova_system SHARED
  src/kinova_system.cpp
)
target_link_libraries(kinova_system PUBLIC kinova_driver_core)
ament_target_dependencies(kinova_system PUBLIC
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface kinova_driver_plugins.xml)

#############
## Install ##
#############

install(TARGETS kinova_driver_core kinova_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Only the headers that compile without ROS 1 are installed. The remaining headers in
# include/kinova_driver belong to code that has not been ported yet.
install(FILES
  include/kinova_driver/conversions.hpp
  include/kinova_driver/fake_kinova_device.hpp
  include/kinova_driver/kinova_api.h
  include/kinova_driver/kinova_comm.h
  include/kinova_driver/kinova_comm_config.hpp
  include/kinova_driver/kinova_device.hpp
  include/kinova_driver/kinova_ros_types.h
  include/kinova_driver/kinova_system.hpp
  DESTINATION include/kinova_driver
)
install(DIRECTORY include/kinova/
  DESTINATION include/kinova
)

# The Kinova .so binary blobs are only available for some architectures. They are installed
# next to the other libraries so a bare-name dlopen finds them through LD_LIBRARY_PATH.
if(CMAKE_LIBRARY_ARCHITECTURE AND EXISTS ${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})
  install(FILES
    lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommLayerUbuntu.so
    lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommandLayerUbuntu.so
    lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommLayerUbuntu.so
    lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommandLayerUbuntu.so
    DESTINATION lib
  )
else()
  message(WARNING "Kinova vendor libraries not available for ${CMAKE_LIBRARY_ARCHITECTURE}; only use_fake_device will work")
endif()

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kinova_api test/test_kinova_api.cpp)
  target_link_libraries(test_kinova_api kinova_driver_core)

  ament_add_gtest(test_kinova_ros_types test/test_kinova_ros_types.cpp)
  target_link_libraries(test_kinova_ros_types kinova_driver_core)

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions kinova_driver_core)

  ament_add_gtest(test_fake_kinova_device test/test_fake_kinova_device.cpp)
  target_link_libraries(test_fake_kinova_device kinova_driver_core)

  ament_add_gtest(test_kinova_system test/test_kinova_system.cpp)
  target_link_libraries(test_kinova_system kinova_system)

  ament_add_gtest(test_kinova_system_services test/test_kinova_system_services.cpp)
  target_link_libraries(test_kinova_system_services kinova_system)

  # Loads the installed plugin through pluginlib; does not link kinova_system.
  ament_add_gtest(test_load_kinova_system test/test_load_kinova_system.cpp)
  ament_target_dependencies(test_load_kinova_system hardware_interface pluginlib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp kinova_msgs geometry_msgs tf2 ament_index_cpp hardware_interface pluginlib rclcpp_lifecycle)
ament_package()
