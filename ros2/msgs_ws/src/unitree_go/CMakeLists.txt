cmake_minimum_required(VERSION 3.8)
project(unitree_go)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

set(Python_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "" FORCE)
set(Python_INCLUDE_DIRS ${Python3_INCLUDE_DIRS} CACHE STRING "" FORCE)
set(Python_LIBRARIES ${Python3_LIBRARIES} CACHE STRING "" FORCE)
set(Python_NumPy_INCLUDE_DIRS ${Python3_NumPy_INCLUDE_DIRS} CACHE STRING "" FORCE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsState.msg"
  "msg/IMUState.msg"
  "msg/LowState.msg"
  "msg/MotorState.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
