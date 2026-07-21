cmake_minimum_required(VERSION 3.8)
project(openleg_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/wasd_ik_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wasd_ik_teleop
)

install(PROGRAMS
  scripts/fixed_gait_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fixed_gait_demo
)
install(PROGRAMS
  scripts/right_leg_lift_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME right_leg_lift_test
)
install(PROGRAMS
  scripts/short_step_walk_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME short_step_walk_demo
)



if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py TIMEOUT 60)
endif()

ament_package()
