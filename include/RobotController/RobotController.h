#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <memory>
#include "Al5dController/Highlevel/HighLevelInterface.h"
#include <actionlib/client/simple_action_client.h>
#include "RobotController/Kinematics.h"
#include "robot_kinematica/found_object.h"

class RobotController
{
public:
  /**
   * @brief Construct a new Robot Controller object
   *
   * @param name of the node
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
   *
   * @internal a signifies half of the x-difference in pixels between the top left and top right corner of the found
   * object
   * @internal b signifies the x-location of the midpoint between the top left and top right corner of the found object
   * @internal c signifies half of the y-difference in pixels between the top left and bottom left corner of the found
   * object
   * @internal d signifies the y-location of the midpoint between the top left and bottom right corner of the found
   * object
   * @internal e signifies the imaginary hypotenuse between point b, point d and the center of the found object
   * @internal f signifies the rotation of the object compared to an imaginary line perpendicular to the x-axis
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
   * @brief is an object which represents the ROS node
   *
   */
  ros::NodeHandle m_node_handle;

  /**
   * @brief boolean wether or not the message has been send
   *
   */
  bool m_objectCoordinatesReceived;

  /**
   * @brief srv message objedt that contains data of the found object
   *
   */
  robot_kinematica::found_object m_found_object;

  /**
   * @brief simple action client object for sending srv messages
   *
   */
  actionlib::SimpleActionClient<robot_kinematica::al5dPositionAction> m_al5d_action_client;

  /**
   * @brief Function for moving the al5d to a requested position
   *
   * @param position The angles the servos should move to
   * @param gripper_angle The angle the gripper must move to to grab the found object
   */
  void moveArm(const std::optional<Kinematics::Matrix<double, 4, 1>> position, const double gripper_angle,
               const std::string goal_name);

  /**
   * @brief Function for moving the gripper
   *
   * @param degrees The amount of degrees the gripper servo should be set to
   */
  void moveGripper(const double degrees);

  /**
   * @brief Function for moving the al5d to its destination
   *
   * @param req request message with information of the object
   * @param res response message which contains if the arm is moved to the position or not
   * @return true when the message is received
   * @return false when the message is not receive
   */
  bool moveObjectToDestination(robot_kinematica::found_object::Request& req,
                               robot_kinematica::found_object::Response& res);

  /**
   * @brief the advertise service object
   *
   */
  ros::ServiceServer m_service;
};

#endif  // ROBOTCONTROLLER_H