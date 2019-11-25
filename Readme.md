## WoR-Robots Application
Detecting and locating squares, rectangles, circles, semi-circles and triangles through OpenCV.
When the object is detected the robot arm moves to the found object and graps it up and places it on
the destination location. These movements are calculated using kinematics based on the found obect and destination.

### Compile instructions:
Place robot_kinematica package in:
```
~/catkin_ws/src/
```
Navigate to catkin_s:
```
cd ~/catkin_ws/
```
Compile with:
```
catkin_make
```
Source workspace:
```
./devel/setup.bash
```

### Usage:
Run application
```
roslaunch robotsapplication application.launch
```