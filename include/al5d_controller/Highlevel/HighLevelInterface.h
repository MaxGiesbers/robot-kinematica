#ifndef HIGHLEVELINTERFACE_H_
#define HIGHLEVELINTERFACE_H

#include "robot_kinematica/al5dPositionAction.h"
#include <actionlib/server/simple_action_server.h>
#include "robot_kinematica/eStop.h"
#include "../LowLevel/LowLevelDriver.h"
#include "robot_kinematica/found_object.h"

#include "Position.h"
#include <vector>

class HighLevelInterface
{
public:
  HighLevelInterface(const std::string& name, const std::string& positions_file_name, const std::string port);
  ~HighLevelInterface(void);
  void executeCB(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  void concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  void concatMessage(const Position& position);
  void initServoList();
  bool emergencyStop(robot_kinematica::eStop::Request& req, robot_kinematica::eStop::Response& res);
  bool parseProgrammedPositions(const std::string& fileName);
  void run(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  void callBack(const robot_kinematica::found_object& found_object);


private:
  ros::NodeHandle m_node_handle;
  actionlib::SimpleActionServer<robot_kinematica::al5dPositionAction> m_al5d_action_server;
  std::string m_name;
  LowLevelDriver m_low_level_component;
  std::vector<Servo> m_servo_list;
  std::vector<Position> m_position_list;
  ros::Subscriber m_subscriber;

};

#endif  // HIGHLEVELINTERFACE_H_
