cmake_minimum_required(VERSION 3.8)
project(kinova_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.dae"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_j2n6s200_xacro test/test_j2n6s200_xacro.py)
endif()

ament_package()
