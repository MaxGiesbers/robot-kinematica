cmake_minimum_required(VERSION 2.8.3)
project(robot_kinematica)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

find_package(
OpenCV 3 REQUIRED)

include_directories(include)


generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  message_runtime
  actionlib
  actionlib_msgs
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