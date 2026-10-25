cmake_minimum_required(VERSION 3.20)
project(mhseals_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Observation.msg
  msg/ObservationArray.msg
  msg/Track.msg
  msg/TrackArray.msg
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

# rosidl already owns ament_python_install_package(mhseals_nav) for the generated msg
# subpackage, and a second call is a hard error, so the sources merge into the same
# install directory and each executable is a generated wrapper. multiprocessing
# "spawn" children re-run the wrapper as __mp_main__, so main() must be guarded or
# each child starts the program again and dies during bootstrap (lidar_relay).
install(DIRECTORY ${PROJECT_NAME}/ DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

set(ENTRY_POINTS
  twist_converter=twist_converter
  object_tracker=object_tracker
  thruster_allocation=thruster_allocation
  zed_backend=zed_backend
  sim_backend=sim_backend
  lidar_relay=lidar_relay
  mavros_odometry=mavros_odometry
  mavros_setup=mavros_setup
  zed_imu_reference=zed_imu_reference
  run_task=tasks.interface
  costmap_watchdog=costmap_watchdog
  approach_guard=approach_guard
  nearest_obstacle=nearest_obstacle
  link_watchdog=link_watchdog
  fence_watchdog=fence_watchdog
  fcu_watchdog=fcu_watchdog
  ekf_input_watchdog=ekf_input_watchdog
)
foreach(entry ${ENTRY_POINTS})
  string(REPLACE "=" ";" entry "${entry}")
  list(GET entry 0 exe)
  list(GET entry 1 module)
  file(GENERATE OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/scripts/${exe}
    CONTENT "#!/usr/bin/python3\nimport sys\nfrom ${PROJECT_NAME}.${module} import main\nif __name__ == '__main__':\n    sys.exit(main())\n"
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/scripts/${exe} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config description rviz2 DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${PROJECT_NAME}/behavior_trees DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
