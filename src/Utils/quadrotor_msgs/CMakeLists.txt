cmake_minimum_required(VERSION 3.16)
project(quadrotor_msgs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/GoalSet.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PPROutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/SO3Command.msg"
  "msg/Serial.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/TakeoffLand.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
add_library(quadrotor_codecs SHARED src/encode_msgs.cpp src/decode_msgs.cpp)
target_include_directories(quadrotor_codecs PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(quadrotor_codecs "${cpp_typesupport_target}" Eigen3::Eigen)
install(TARGETS quadrotor_codecs EXPORT codecsTargets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(codecsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime Eigen3)

ament_package()
