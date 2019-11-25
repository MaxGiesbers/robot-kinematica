#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include "al5d_controller/Highlevel/HighLevelInterface.h"
#include <actionlib/client/simple_action_client.h>
#include "al5d_controller/Kinematics.h"
#include "robot_kinematica/found_object.h"

class RobotController
{
public:
  /**
   * @brief Construct a new Robot Controller object
   * 
   * @param name 
   */
  RobotController(const std::string& name);
  
  /**
   * @brief Destroy the Robot Controller object
   * 
   */
  ~RobotController();

private:
  /**
   * @brief Create a new instance of the kinematics class
   * 
   */
  Kinematics::Kinematics m_kinematics;
  
  /**
   * @brief A matrix containing the angles the servos are currently set to
   * 
   */
  Kinematics::Matrix<double, 4, 1> m_current_angles;
  
  /**
   * @brief Get the the angle the gripper should be set to
   * 
   * @param req Message containing various required specifications of the found object 
   * @param position The angles the servos should be set to to grab the found object
   * @return double The amount of degrees the gripper (wristRotate) should be set to
   */
  double getGripperAngle(robot_kinematica::found_object::Request& req,
                         const std::optional<Kinematics::Matrix<double, 4, 1>> position);
  
  /**
   * @brief Function for correcting the getGripperAngle value for the wristRotate limits
   * 
   * @param angle Calculated angle
   * @return double Corrected angle
   */
  double correctForServoLimits(double angle);
  
  /**
   * @brief 
   * 
   */
  ros::NodeHandle m_node_handle;
  
  /**
   * @brief 
   * 
   */
  bool m_objectCoordinatesReceived;
  
  /**
   * @brief 
   * 
   */
  robot_kinematica::found_object m_found_object;
  
  /**
   * @brief 
   * 
   */
  actionlib::SimpleActionClient<robot_kinematica::al5dPositionAction> m_al5d_action_client;
  
  /**
   * @brief Function for moving the al5d to a requested position
   * 
   * @param position The angles the servos should move to
   * @param gripper_angle The angle the gripper must move to to grab the found object
   */
  void moveArm(const std::optional<Kinematics::Matrix<double, 4, 1>> position, const double gripper_angle);
  
  /**
   * @brief Function for moving the gripper
   * 
   * @param degrees The amount of degrees the gripper servo should be set to
   */
  void moveGripper(const double degrees);
  
  /**
   * @brief Function for moving the al5d to its destination
   * 
   * @param req 
   * @param res 
   * @return true 
   * @return false 
   */
  bool moveObjectToDestination(robot_kinematica::found_object::Request& req,
                               robot_kinematica::found_object::Response& res);
  
  /**
   * @brief 
   * 
   */
  ros::ServiceServer m_service;
};

#endif  // ROBOTCONTROLLER_H