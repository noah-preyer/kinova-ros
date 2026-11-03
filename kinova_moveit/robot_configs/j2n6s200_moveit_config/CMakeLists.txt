cmake_minimum_required(VERSION 3.8)
project(j2n6s200_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_srdf_consistency test/test_srdf_consistency.py TIMEOUT 60)
  add_launch_test(test/test_move_group_fake.py TIMEOUT 180)
endif()

ament_package()
