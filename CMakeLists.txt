cmake_minimum_required(VERSION 2.8.3)
project(robot_kinematica)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  control_msgs
  rospy
)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})


find_package (OpenCV 3 REQUIRED)
include_directories(include)


add_service_files(FILES
  eStop.srv
  found_object1.srv
)

add_action_files(
  DIRECTORY action
  FILES 
  al5dPosition.action
)

add_message_files(
  FILES
  found_object.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
  control_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  message_runtime
  LIBRARIES 
  al5d_driver
  actionlib
  actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})



add_compile_options(-std=c++17 -Wall -Wextra)

add_executable(vision 
         src/vision/VisionController.cpp
         src/vision/ColorObject.cpp
         src/vision/Calibration.cpp
	       src/vision/ObjectDetector.cpp)
add_dependencies(vision robot_kinematica_generate_messages_cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


add_library(al5d_driver 
    src/al5d_controller/Lowlevel/Servo.cpp 
    src/al5d_controller/Lowlevel/LowLevelDriver.cpp)
target_link_libraries(al5d_driver ${catkin_LIBRARIES})

add_executable(al5d_interface 
          src/al5d_controller/Highlevel/HighLevelInterface.cpp)
target_link_libraries(al5d_interface al5d_driver ${catkin_LIBRARIES})
add_dependencies(al5d_interface al5d_driver robot_kinematica_generate_messages_cpp)

add_executable(robot_controller 
          src/al5d_controller/RobotController.cpp
          src/al5d_controller/Kinematics.cpp)
target_link_libraries(robot_controller ${catkin_LIBRARIES})
add_dependencies(robot_controller robot_kinematica_generate_messages_cpp)
