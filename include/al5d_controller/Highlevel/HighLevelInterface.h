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
   * @param name the name of the node
   * @param port the name of the usb port that is connected to the arm
   */
  HighLevelInterface(const std::string& name, const std::string port);
  
  /**
   * @brief Destroy the High Level Interface object
   * 
   */
  ~HighLevelInterface(void);
  
  /**
   * @brief Callback that is called when an message is received from the robotcontroller
   * 
   * @param goal contains the coordinates for the position of the movement
   */
  void executeCB(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  
  /**
   * @brief concatenates the received message and prepares it for sending to the arm
   * 
   * @param goal contains the coordinates for the position of the movement
   */
  void concatMessage(const robot_kinematica::al5dPositionGoalConstPtr& goal);
  
  /**
   * @brief starts the movement when an message has received
   * 
   * @param goal contains the coordinates for the position of the movement
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
   * @brief Moves all the servo of the al5d controller
   * 
   */
  void moveAllServos();
  
  /**
   * @brief initialize all the servo's
   * 
   */
  void initServoList();
  
  /**
   * @brief Moves only the servo's that has been set
   * 
   */
  void moveServos();

private:
  /**
   * @brief is an object which represents the ROS node
   * 
   */
  ros::NodeHandle m_node_handle;

  /**
   * @brief actionlib server object for receiving and sending information
   * 
   */
  actionlib::SimpleActionServer<robot_kinematica::al5dPositionAction> m_al5d_action_server;

  /**
   * @brief name of the node
   * 
   */
  std::string m_name;

  /**
   * @brief contains the lowlevel logica of the al5d arm
   * 
   */
  LowLevelDriver m_low_level_component;

  /**
   * @brief list of servo's of the al5d arm
   * 
   */
  std::vector<Servo> m_servo_list;
};

#endif  // HIGHLEVELINTERFACE_H_
