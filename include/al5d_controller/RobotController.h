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
  RobotController(const std::string& name);
  ~RobotController();

private:
  Kinematics::Kinematics m_kinematics;
  Kinematics::Matrix<double, 4, 1> m_current_angles;
  double getGripperAngle(robot_kinematica::found_object::Request& req,
                         const std::optional<Kinematics::Matrix<double, 4, 1>> position);
  double correctForServoLimits(double angle);
  ros::NodeHandle m_node_handle;
  bool m_objectCoordinatesReceived;
  robot_kinematica::found_object m_found_object;
  actionlib::SimpleActionClient<robot_kinematica::al5dPositionAction> m_al5d_action_client;
  void moveArm(const std::optional<Kinematics::Matrix<double, 4, 1>> position, const double gripper_angle);
  void moveGripper(const double degrees);
  bool moveObjectToDestination(robot_kinematica::found_object::Request& req,
                               robot_kinematica::found_object::Response& res);
  ros::ServiceServer m_service;
};

#endif  // ROBOTCONTROLLER_H