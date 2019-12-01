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
  found_object.srv
)

add_action_files(
  DIRECTORY action
  FILES 
  al5dPosition.action
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

add_executable(VisionController 
         src/VisionController/VisionController.cpp
         src/VisionController/ColorObject.cpp
         src/VisionController/Calibration.cpp
	       src/VisionController/ObjectDetector.cpp)
add_dependencies(VisionController robot_kinematica_generate_messages_cpp)
target_link_libraries(VisionController ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


add_library(al5d_driver 
    src/Al5dController/Lowlevel/Servo.cpp 
    src/Al5dController/Lowlevel/LowLevelDriver.cpp)
target_link_libraries(al5d_driver ${catkin_LIBRARIES})

add_executable(Al5dController 
          src/Al5dController/Highlevel/HighLevelInterface.cpp)
target_link_libraries(Al5dController al5d_driver ${catkin_LIBRARIES})
add_dependencies(Al5dController al5d_driver robot_kinematica_generate_messages_cpp)

add_executable(RobotController 
          src/RobotController/RobotController.cpp
          src/RobotController/Kinematics.cpp)
target_link_libraries(RobotController ${catkin_LIBRARIES})
add_dependencies(RobotController robot_kinematica_generate_messages_cpp)
