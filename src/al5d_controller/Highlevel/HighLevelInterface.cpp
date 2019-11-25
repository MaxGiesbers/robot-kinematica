#include "al5d_controller/Highlevel/HighLevelInterface.h"
#include <fstream>

HighLevelInterface::HighLevelInterface(const std::string& name, const std::string port)
  : m_al5d_action_server(m_node_handle, name, boost::bind(&HighLevelInterface::executeCB, this, _1), false)
  , m_name(name)
  , m_low_level_component(port)
{
  initServoList();
  m_al5d_action_server.start();
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
  if (goal->name.compare("PARK") == 0)
  {
    park();
  }
  else
  {
    concatMessage(goal);
  }
}

void HighLevelInterface::openGripper()
{
  m_servo_list[SERVO_ID::GRIPPER].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::GRIPPER].setMoveServo(true);
  moveServos();
}

void HighLevelInterface::closeGripper()
{
  m_servo_list[SERVO_ID::GRIPPER].setIncomingDegrees(29);
  m_servo_list[SERVO_ID::GRIPPER].setMoveServo(true);
  moveServos();
}

void HighLevelInterface::park()
{
  m_servo_list[SERVO_ID::BASE].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::SHOULDER].setIncomingDegrees(-30);
  m_servo_list[SERVO_ID::ELBOW].setIncomingDegrees(115);
  m_servo_list[SERVO_ID::WRIST].setIncomingDegrees(-55);
  m_servo_list[SERVO_ID::WRIST_ROTATE].setIncomingDegrees(0);
  m_servo_list[SERVO_ID::GRIPPER].setIncomingDegrees(0);
  moveAllServos();
  moveServos();
}

void HighLevelInterface::moveAllServos()
{
  for (Servo& servo : m_servo_list)
  {
    servo.setMoveServo(true);
  }
}

void HighLevelInterface::moveServos()
{
  std::stringstream ss;
  for (Servo& servo : m_servo_list)
  {
    if (servo.canMoveServo())
    {
      ss << "#" << servo.getServoId() << "P" << servo.degreesToPwm(servo.getIncomingDegrees());
      servo.setMoveServo(false);
    }
  }
  ss << "T" << 2000 << "\r";
  m_low_level_component.writeMessage(ss.str());
  m_al5d_action_server.setSucceeded();
}

void HighLevelInterface::concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal)
{
  std::stringstream ss;

  for (std::size_t i = 0; i < (*goal).servos.size(); i++)
  {
    Servo& servo = m_servo_list[(*goal).servos[i]];
    servo.setIncomingDegrees((*goal).degrees[i]);
    ss << "#" << servo.getServoId() << "P" << servo.degreesToPwm(servo.getIncomingDegrees());
  }

  ss << "T" << 2000 << "\r";
  m_low_level_component.writeMessage(ss.str());

  ros::Duration(3).sleep();

  m_al5d_action_server.setSucceeded();
  ROS_INFO_STREAM("STATE: SUCCEEDED: " << (*goal).name);
}

void HighLevelInterface::initServoList()
{
  m_servo_list.push_back(Servo(SERVO_ID::BASE, -90, 90, 553, 2425));
  m_servo_list.push_back(Servo(SERVO_ID::SHOULDER, -30, 90, 1798, 556));
  m_servo_list.push_back(Servo(SERVO_ID::ELBOW, 0, 135, 679, 1929));
  m_servo_list.push_back(Servo(SERVO_ID::WRIST, -90, 90, 553, 2520));
  m_servo_list.push_back(Servo(SERVO_ID::WRIST_ROTATE, -90, 90, 600, 2400));
  m_servo_list.push_back(Servo(SERVO_ID::GRIPPER, 0, 29, 1000, 2100));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "al5d_interface");

  if (argc != 2)
  {
    ROS_WARN_STREAM("Missing USB port argument trye something like: /dev/ttyUSB0");
    return 1;
  }

  try
  {
    HighLevelInterface highLevelInterface("robot_kinematica", argv[1]);
    ros::spin();
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
