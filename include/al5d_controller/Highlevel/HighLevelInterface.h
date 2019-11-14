#ifndef HIGHLEVELINTERFACE_H_
#define HIGHLEVELINTERFACE_H

#include "robot_kinematica/al5dPositionAction.h"
#include <actionlib/server/simple_action_server.h>
#include "robot_kinematica/eStop.h"
#include "../LowLevel/LowLevelDriver.h"
#include "robot_kinematica/found_object.h"

#include <vector>

class HighLevelInterface
{
public:
  HighLevelInterface(const std::string& name, const std::string port);
  ~HighLevelInterface(void);
  void executeCB(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  void concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  void run(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  void openGripper();
  void closeGripper();
  void park();
  void moveAllServos();
  void initServoList();
  void moveServos();

private:
  ros::NodeHandle m_node_handle;
  actionlib::SimpleActionServer<robot_kinematica::al5dPositionAction> m_al5d_action_server;
  std::string m_name;
  LowLevelDriver m_low_level_component;
  std::vector<Servo> m_servo_list;
};

#endif  // HIGHLEVELINTERFACE_H_
