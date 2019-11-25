#ifndef HIGHLEVELINTERFACE_H
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
  /**
   * @brief Construct a new High Level Interface object
   * 
   * @param name 
   * @param port 
   */
  HighLevelInterface(const std::string& name, const std::string port);
  
  /**
   * @brief Destroy the High Level Interface object
   * 
   */
  ~HighLevelInterface(void);
  
  /**
   * @brief 
   * 
   * @param goal 
   */
  void executeCB(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  
  /**
   * @brief 
   * 
   * @param goal 
   */
  void concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  
  /**
   * @brief 
   * 
   * @param goal 
   */
  void run(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  
  /**
   * @brief Function for setting the gripper servo to the open position
   * 
   */
  void openGripper();
  
  /**
   * @brief Function for setting the gripper servo to the closed position
   * 
   */
  void closeGripper();
  
  /**
   * @brief Function for making the al5d move to the park position
   * 
   */
  void park();
  
  /**
   * @brief 
   * 
   */
  void moveAllServos();
  
  /**
   * @brief 
   * 
   */
  void initServoList();
  
  /**
   * @brief 
   * 
   */
  void moveServos();

private:
  /**
   * @brief 
   * 
   */
  ros::NodeHandle m_node_handle;

  /**
   * @brief 
   * 
   */
  actionlib::SimpleActionServer<robot_kinematica::al5dPositionAction> m_al5d_action_server;

  /**
   * @brief 
   * 
   */
  std::string m_name;

  /**
   * @brief 
   * 
   */
  LowLevelDriver m_low_level_component;

  /**
   * @brief 
   * 
   */
  std::vector<Servo> m_servo_list;
};

#endif  // HIGHLEVELINTERFACE_H_
