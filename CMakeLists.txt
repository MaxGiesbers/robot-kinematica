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

add_service_files(FILES
  eStop.srv
)

add_action_files(
  DIRECTORY action
  FILES 
  al5dPosition.action
)

find_package(
OpenCV 3 REQUIRED)

include_directories(include)


generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs 
  control_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  message_runtime
  actionlib
  actionlib_msgs
  LIBRARIES 
  al5d_driver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++17 -Wall -Wextra)


add_executable(vision 
         src/vision/VisionController.cpp
         src/vision/ColorObject.cpp
         src/vision/Calibration.cpp
	       src/vision/ObjectDetector.cpp)
add_dependencies(vision robotsVisionController_generate_messages_cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

include_directories(${catkin_INCLUDE_DIRS})



add_library(al5d_driver 
    src/al5d_controller/Lowlevel/Servo.cpp 
    src/al5d_controller/Lowlevel/SerialCommunication.cpp 
    src/al5d_controller/Lowlevel/LowLevelDriver.cpp)
target_link_libraries(al5d_driver ${catkin_LIBRARIES})

add_executable(al5d_interface 
          src/al5d_controller/Highlevel/HighLevelInterface.cpp 
          src/al5d_controller/Highlevel/Position.cpp)
target_link_libraries(al5d_interface al5d_driver ${catkin_LIBRARIES})

add_executable(al5d_motionplanner 
          src/al5d_controller/Motionplanner/MotionPlanner.cpp)
target_link_libraries(al5d_motionplanner ${catkin_LIBRARIES})


add_dependencies(al5d_motionplanner al5d_controller_generate_messages_cpp)
add_dependencies(al5d_interface al5d_driver al5d_controller_generate_messages_cpp)

