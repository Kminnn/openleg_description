cmake_minimum_required(VERSION 3.8)
project(openleg_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/wasd_ik_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wasd_ik_teleop
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py TIMEOUT 60)
endif()

ament_package()
