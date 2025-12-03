
cmake_minimum_required(VERSION 3.8)
project(dls2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
  "msg/Screw.msg"
  "msg/BlindState.msg"
  "msg/BaseState.msg"
  "msg/Imu.msg"
  "msg/FeetContactState.msg"
  "msg/TrajectoryGenerator.msg"
  "msg/TimeDebug.msg"
  "msg/ControlSignal.msg"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()