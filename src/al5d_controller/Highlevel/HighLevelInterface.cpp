#include "al5d_controller/Highlevel/HighLevelInterface.h"
#include <fstream>

namespace
{
const uint8_t MINIMAL_ARGUMENTS = 4;
}
HighLevelInterface::HighLevelInterface(const std::string& name, const std::string& positions_file_name,
                                       const std::string port)
  : m_al5d_action_server(m_node_handle, name, boost::bind(&HighLevelInterface::executeCB, this, _1), false)
  , m_name(name)
  , m_low_level_component(port)
{
  initServoList();
  m_al5d_action_server.start();
  parseProgrammedPositions(positions_file_name);
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
  auto position = std::find_if(m_position_list.begin(), m_position_list.end(),
                               [&](const Position pos) { return pos.getName().compare(goal->name) == 0; });

  if (position != std::end(m_position_list))
  {
    concatMessage((*position));
  }
  else
  {
    concatMessage(goal);
  }
}

bool HighLevelInterface::emergencyStop(robot_kinematica::eStop::Request& req, robot_kinematica::eStop::Response& res)
{
  m_low_level_component.emergencyStop();
  return true;
}

void HighLevelInterface::concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal)
{
  std::vector<int> positions;
  for (std::size_t i = 0; i < (*goal).degrees.size(); ++i)
  {
    int pw = m_servo_list.at(i).degreesToPwm((*goal).degrees[i]);
    positions.push_back(pw);
  }
  m_low_level_component.goToPosition(positions, (*goal).time);

  ros::Duration(4).sleep();
  m_al5d_action_server.setSucceeded();
  ROS_INFO_STREAM("STATE: SUCCEEDED: " << (*goal).name);
}

void HighLevelInterface::concatMessage(const Position& position)
{
  std::vector<int> positions;

  for (std::size_t i = 0; i < position.getDegrees().size(); ++i)
  {
    int pw = m_servo_list.at(i).degreesToPwm(position.getDegrees()[i]);
    positions.push_back(pw);
  }
  m_low_level_component.goToPosition(positions, position.getTime());

  ros::Duration(4).sleep();
  m_al5d_action_server.setSucceeded();
  ROS_INFO_STREAM("STATE: SUCCEEDED: " << position.getName());
}

void HighLevelInterface::initServoList()
{
  m_servo_list.push_back(Servo("Base", -90, 90, 500, 2500));
  m_servo_list.push_back(Servo("Shoulder", -30, 90, 1100, 2500));
  m_servo_list.push_back(Servo("Elbow", 0, 135, 650, 2500));
  m_servo_list.push_back(Servo("Wrist", -90, 90, 500, 2500));
  m_servo_list.push_back(Servo("WristRotate", -90, 90, 500, 2500));
  m_servo_list.push_back(Servo("Gripper", -90, 90, 500, 2500));
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
    name = value;
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "al5d_interface");
  if (argc != MINIMAL_ARGUMENTS)
  {
    ROS_WARN_STREAM("Missing arguments. Try something like: rosrun robot_kinematica al5d_interface "
                    "ProgrammedPositions.csv /dev/ttyUSB0");
    return 1;
  }

  HighLevelInterface highLevelInterface("robot_kinematica", argv[2], argv[3]);
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("eStop", &HighLevelInterface::emergencyStop, &highLevelInterface);
  ros::spin();
  return 0;
}
