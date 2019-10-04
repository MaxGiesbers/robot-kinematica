# world-simulation-bp2

Start simulatie commando:
roslaunch al5d_simulation startup.launch 

Bug geen robot commando: 

export LC_NUMERIC="en_US.UTF-8"



path to text file: 
Run publisher

rosrun al5d_simulation demo_publish_node src/world-simulation-bp2/al5d_simulation/demo/demo_commands.txt

Virtual controller needs arguments for position of the arm.
rosrun al5d_simulation VirtualController 0.0 0.0 0.0



gripper lijkt andere waardes te hebben beginnend van 1500 tot 1535 ongeveer.


for debugging:
std::cout << "right y: " << std::abs(diff_right_y) << std::endl;
std::cout <<  "left y: " << std::abs(diff_left_y) << std::endl;
std::cout << "right x: " << std::abs(diff_right_x)<< std::endl;
std::cout <<  "left x: " << std::abs(diff_left_x) << std::endl;
std::cout << "right z: " << std::abs(diff_right_z) << std::endl;
std::cout <<  "left z: " << std::abs(diff_left_z) << std::endl;

info over de servos
// servoList.push_back(Lowlevel::Servo("Base", -90, 90, 500, 2500));
    // servoList.push_back(Lowlevel::Servo("Shoulder", -30, 90, 1100, 2500));
    // servoList.push_back(Lowlevel::Servo("Elbow", 0, 135, 650, 2500));
    // servoList.push_back(Lowlevel::Servo("Wrist", -90, 90, 500, 2500));
    // servoList.push_back(Lowlevel::Servo("WristRotate", -90, 90, 500, 2500));
    // servoList.push_back(Lowlevel::Servo("Gripper", -90, 90, 500, 2500));



clang-format-3.8 -i -style=file MY_ROS_NODE.cpp

find . -name '*.h' -or -name '*.h' -or -name '*.cpp' | xargs clang-format -i -style=file $1



# robot-kinematica
