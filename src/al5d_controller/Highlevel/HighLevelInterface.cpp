#include "al5d_controller/Highlevel/HighLevelInterface.h"
#include <fstream>

HighLevelInterface::HighLevelInterface(const std::string& name, const std::string& positions_file_name,
                                       const std::string port)
  : m_al5d_action_server(m_node_handle, name, boost::bind(&HighLevelInterface::executeCB, this, _1), false)
  , m_name(name)
  , m_low_level_component(port)
{
  ros::ServiceServer service = m_node_handle.advertiseService("eStop", &HighLevelInterface::emergencyStop, this);

  initServoList();
  m_al5d_action_server.start();


  // m_low_level_component.writeMessage("#0P1500S500#1P1998S500#2P1992S500#3P1000S500#4P1500S500#5P1000S500\r");
  
 // park();
  openGripper();
  // parseProgrammedPositions(positions_file_name);
}
HighLevelInterface::~HighLevelInterface()
{
}

void HighLevelInterface::executeCB(const robot_kinematica::al5dPositionGoalConstPtr& goal)
{
  ROS_INFO("STATE: HANDLE GOAL: %s", (*goal).name.c_str());
  if (!ros::ok())
  {
    m_al5d_action_server.setAborted();
    ROS_INFO("%s Shutting down", m_name.c_str());
  }
  else
  {
    run(goal);
  }
}

void HighLevelInterface::run(const robot_kinematica::al5dPositionGoalConstPtr& goal)
{
  concatMessage(goal);
}

bool HighLevelInterface::emergencyStop(robot_kinematica::eStop::Request& req, robot_kinematica::eStop::Response& res)
{
  m_low_level_component.emergencyStop();
  return true;
}

void HighLevelInterface::openGripper()
{
  m_servo_list[SERVO_ID::GRIPPER].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::GRIPPER].setMoveServo(true);
  std::cout << m_servo_list[SERVO_ID::GRIPPER].getServoId() << std::endl;
  std::cout << m_servo_list[SERVO_ID::GRIPPER].getIncomingDegrees() << std::endl;
  moveServos();
}

void HighLevelInterface::ready()
{
  m_servo_list[SERVO_ID::BASE].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::SHOULDER].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::ELBOW].setIncomingDegrees(10);
  m_servo_list[SERVO_ID::WRIST].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::WRIST_ROTATE].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::GRIPPER].setIncomingDegrees(0);
  moveAllServos();
  moveServos();
}

void HighLevelInterface::park()
{
  std::cout << "park" << std::endl;
  m_servo_list[SERVO_ID::BASE].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::SHOULDER].setIncomingDegrees(47);
  m_servo_list[SERVO_ID::ELBOW].setIncomingDegrees(98);
  m_servo_list[SERVO_ID::WRIST].setIncomingDegrees(-45);
  m_servo_list[SERVO_ID::WRIST_ROTATE].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::GRIPPER].setIncomingDegrees(0);
  moveAllServos();
  moveServos();
}

void HighLevelInterface::straight()
{
  m_servo_list[SERVO_ID::BASE].setCurrentDegrees(0);
  m_servo_list[SERVO_ID::SHOULDER].setCurrentDegrees(40);
  m_servo_list[SERVO_ID::ELBOW].setCurrentDegrees(80);
  m_servo_list[SERVO_ID::WRIST].setCurrentDegrees(5);
  m_servo_list[SERVO_ID::WRIST_ROTATE].setCurrentDegrees(0);
  m_servo_list[SERVO_ID::GRIPPER].setCurrentDegrees(0);
  moveAllServos();
  moveServos();
}

void HighLevelInterface::moveAllServos()
{
  for (Servo& servo: m_servo_list)
  {
    servo.setMoveServo(true);
  }
}


// std::stringstream ss;
//   for (Servo& servo: m_servo_list)
//   {
//     if (servo.canMoveServo())
//     {
//       ss << "#" << servo.getServoId() << "P" << servo.degreesToPwm(servo.getIncomingDegrees());
//       servo.setCurrentDegrees(servo.getIncomingDegrees()); //maybe delete
//       servo.setMoveServo(false);
//     }



void HighLevelInterface::moveServos()
{

  std::cout << m_servo_list[SERVO_ID::BASE].getServoId() << std::endl;
  std::cout << m_servo_list[SERVO_ID::BASE].canMoveServo() << std::endl;
  std::stringstream ss;
  for (Servo& servo : m_servo_list)
  {
    if(servo.canMoveServo())
    {
      std::cout << "komt toch hier in" << std::endl;
      std::cout << servo.getServoId() << std::endl;
      std::cout << servo.getIncomingDegrees() << std::endl;


      ss << "#" << servo.getServoId() << "P" << servo.degreesToPwm(servo.getIncomingDegrees());
      servo.setMoveServo(false);
    }
  }
  std::cout << ss.str() << std::endl;
  ss << "T" << 2000 << "\r";
  m_low_level_component.writeMessage(ss.str());
  // ros::Duration(4).sleep();
  // m_al5d_action_server.setSucceeded();
}


void HighLevelInterface::concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal)
{
  std::stringstream ss;
  for (int servo_number: (*goal).servos)
  {
    
    Servo& servo = m_servo_list[servo_number];
    servo.setIncomingDegrees((*goal).degrees[servo_number]);
    ss << "#" << servo.getServoId() << "P" << servo.degreesToPwm(servo.getIncomingDegrees());
  }

  ss << "T" << 2000 << "\r";
  m_low_level_component.writeMessage(ss.str());

  ros::Duration(4).sleep();
  m_al5d_action_server.setSucceeded();
  ROS_INFO_STREAM("STATE: SUCCEEDED: " << (*goal).name);

  // for (std::size_t i = 0; i < (*goal).servos.size(); ++i)
  // {
  //   if(m_servo_list[i].getServoId() == 

  // }
  // for (auto degrees: (*goal).degrees)
  // {
  //   for (auto servos: (*goal).servos)
  //   {

  //   }
  // }
  // for (std::size_t i = 0; i < (*goal).degrees.size(); ++i)
  // {
  //   //int pw = m_servo_list.at(i).degreesToPwm((*goal).degrees[i]);
  //   //positions.push_back(pw);
  // }
  // //m_low_level_component.goToPosition(positions, (*goal).time);

  // ros::Duration(4).sleep();
  // m_al5d_action_server.setSucceeded();
  // ROS_INFO_STREAM("STATE: SUCCEEDED: " << (*goal).name);
}


void HighLevelInterface::initServoList()
{
  m_servo_list.push_back(Servo(SERVO_ID::BASE, -90, 90, 500, 2500));
  m_servo_list.push_back(Servo(SERVO_ID::SHOULDER, -30, 90, 1100, 2500));
  m_servo_list.push_back(Servo(SERVO_ID::ELBOW, 0, 135, 650, 2500));
  m_servo_list.push_back(Servo(SERVO_ID::WRIST, -90, 90, 500, 2500));
  m_servo_list.push_back(Servo(SERVO_ID::WRIST_ROTATE, -90, 90, 500, 2500));
  m_servo_list.push_back(Servo(SERVO_ID::GRIPPER, 0, 29, 1000, 2100));
}

void HighLevelInterface::concatMessage(const Position& position)
{
  std::vector<int> positions;

  for (std::size_t i = 0; i < position.getDegrees().size(); ++i)
  {
    //int pw = m_servo_list.at(i).degreesToPwm(position.getDegrees()[i]);
    
    //positions.push_back(pw);
  }
  //m_low_level_component.goToPosition(positions, position.getTime());

  ros::Duration(4).sleep();
  m_al5d_action_server.setSucceeded();
  ROS_INFO_STREAM("STATE: SUCCEEDED: " << position.getName());
}



bool HighLevelInterface::parseProgrammedPositions(const std::string& fileName)
{
  std::ifstream file(fileName);
  if (!file.is_open())
  {
    std::cout << "There are no preprogrammed positions in " << fileName << std::endl;
    return false;
  }
  std::string value;
  char spacer = ',';

  // Get rid of the first line with information headers
  std::getline(file, value, '\n');

  while (file.good())
  {
    // init all variables from file
    std::string name = "";
    int degreesBase, degreesShoulder, degreesElbow, degreesWrist, degreesWristRotate, degreesGripper, time = 0;

    // name
    std::getline(file, value, spacer);
    // DEGREESBASE
    std::getline(file, value, spacer);
    degreesBase = std::stoi(value);
    // DEGREESSHOULDER
    std::getline(file, value, spacer);
    degreesShoulder = std::stoi(value);
    // DEGREESELBOW
    std::getline(file, value, spacer);
    degreesElbow = std::stoi(value);
    // DEGREESWRIST
    std::getline(file, value, spacer);
    degreesWrist = std::stoi(value);
    // DEGREESWRISTROTATE
    std::getline(file, value, spacer);
    degreesWristRotate = std::stoi(value);
    // DEGREESGRIPPER
    std::getline(file, value, spacer);
    degreesGripper = std::stoi(value);
    // TIME
    std::getline(file, value, spacer);
    time = std::stoi(value);
    // END OF LINE
    std::getline(file, value, '\n');

    // put the variables in list of Positions
    Position position;

    position.setName(name);
    position.addDegrees(degreesBase);
    position.addDegrees(degreesShoulder);
    position.addDegrees(degreesElbow);
    position.addDegrees(degreesWrist);
    position.addDegrees(degreesWristRotate);
    position.addDegrees(degreesGripper);
    position.setTime(time);
    m_position_list.push_back(position);
  }
  file.close();
  return true;
}