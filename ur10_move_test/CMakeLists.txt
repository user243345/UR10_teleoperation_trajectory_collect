cmake_minimum_required(VERSION 3.0.2)
project(ur10_move_test)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  diagnostic_msgs
  gazebo_msgs
  geometry_msgs
  message_generation
  moveit_msgs
  robotiq_ft_sensor
  roscpp
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  tf2_geometry_msgs
  trajectory_msgs
  cv_bridge
  image_transport
  rosconsole
)

find_package(Eigen3 REQUIRED CONFIG)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

#find_package(Boost REQUIRED COMPONENTS system)
add_message_files(
  FILES
  # OmniButtonEvent.msg
  # Robotiq2FGripper_robot_input.msg
  # Robotiq2FGripper_robot_output.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  # OmniButtonEvent.msg
  # Robotiq2FGripper_robot_input.msg
  # Robotiq2FGripper_robot_output.msg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_planning
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add_executable(ur10_main 
#                       src/ur10_main.cpp
#                       src/ur10_inverse.cpp
#                       src/ur10_move.cpp
#                       src/ur10_topic_subscriber.cpp
#                       src/ur10_teleoperation.cpp
#                       )
add_executable(ur10_trajectory_collect 
                      src/ur10_trajectory_collect.cpp
                      src/ur10_inverse.cpp
                      src/ur10_move.cpp
                      src/ur10_topic_subscriber.cpp
                      src/ur10_teleoperation.cpp
                      src/ur10_gripper.cpp
                      )
add_executable(gripper_test 
                      src/gripper_test.cpp
                      src/ur10_gripper.cpp
                      )
add_executable(ur10_collect 
                      src/ur10_collect.cpp
                      src/ur10_inverse.cpp
                      src/ur10_move.cpp
                      src/ur10_topic_subscriber.cpp
                      src/ur10_teleoperation.cpp
                      src/ur10_gripper.cpp
                      )             
add_executable(image_test
                      src/image_test.cpp)
# add_executable(image_test_update
#                       src/image_test_update.cpp)
# target_link_libraries(ur10_main
#   ${catkin_LIBRARIES}
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_LIBRARIES}
# )
target_link_libraries(ur10_trajectory_collect
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_LIBRARIES}
)
target_link_libraries(gripper_test
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_LIBRARIES}
)
target_link_libraries(ur10_collect
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_LIBRARIES}
)
target_link_libraries(image_test
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBS}
)
# target_link_libraries(image_test_update
#   ${catkin_LIBRARIES}
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_LIBRARIES}
# )
# set(PYTHONPATH ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)
catkin_install_python(PROGRAMS
  scripts/test.py
  scripts/test_torch.py
  scripts/ur10_imitation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


