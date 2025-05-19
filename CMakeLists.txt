cmake_minimum_required(VERSION 3.16)

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
set(MSVC_INCREMENTAL_DEFAULT ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(cv_project)

# find_package(mujoco REQUIRED)
# include_directories("~/mujoco/mujoco-3.2.0/include")
# link_directories("~/mujoco/mujoco-3.2.0/lib")
include_directories("$ENV{HOME}/mujoco/mujoco-3.3.1/include")
link_directories("$ENV{HOME}/mujoco/mujoco-3.3.1/lib")

find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  std_msgs roscpp geometry_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

# messages
add_message_files(
  DIRECTORY msg
  FILES
  robotWrench.msg
  setpoint.msg
  robotState.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

# executable for simulator
add_executable(simulator "")
target_sources(
  simulator
  PRIVATE
  src/simulator/main.cpp
  src/simulator/simulator.cpp
  src/simulator/robot.cpp
  src/simulator/ros_io/measurement_publisher.cpp
  src/simulator/ros_io/setpoint_subscriber.cpp
  src/simulator/ros_io/state_publisher.cpp
  src/utils/config.cpp
  src/utils/data_struct.cpp
  src/utils/math.cpp
)
target_include_directories(
  simulator
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src
)
target_link_libraries(
  simulator
  PRIVATE
  ${catkin_LIBRARIES}
  yaml-cpp
  mujoco
  glfw
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
target_compile_features(
  simulator
  PRIVATE
  cxx_std_17
)

# Install Python scripts and make them executable
catkin_install_python(
  PROGRAMS
    scripts/trajectory_generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

