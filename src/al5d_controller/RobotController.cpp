#include "al5d_controller/RobotController.h"
#include "matrix_class/Matrix.hpp"
#include "ros/ros.h"

namespace
{
const uint8_t MINIMAL_ARGUMENTS = 4;
const uint16_t QUEUE_SIZE = 1000;
}  // namespace

RobotController::RobotController(const std::string& name)
  : m_objectCoordinatesReceived(false), m_al5d_action_client(m_node_handle, name)
{
  m_service = m_node_handle.advertiseService("found_object", &RobotController::moveObjectToDestination, this);
}

bool RobotController::moveObjectToDestination(robot_kinematica::found_object::Request& req,
                                              robot_kinematica::found_object::Response& res)
{
  m_current_angles[0][0] = 0.0;
  m_current_angles[0][1] = -30.0;
  m_current_angles[0][2] = 115.0;
  m_current_angles[0][3] = -55.0;

  const Kinematics::Matrix<double, 3, 1> object_position(
      { { req.origin_y }, { req.origin_z }, { req.origin_x - 0.015 } });

  const Kinematics::Matrix<double, 3, 1> object_position_below(
      { { req.origin_y }, { req.origin_z - 0.05 }, { req.origin_x - 0.015 } });

  const Kinematics::Matrix<double, 3, 1> destination_position(
      { { req.destination_y }, { 0 }, { req.destination_x - 0.015 } });

  auto above_object_position = m_kinematics.inverse_kinematics(m_current_angles, object_position);
  auto above_object_grap_position = m_kinematics.inverse_kinematics(m_current_angles, object_position_below);
  auto above_destination_position = m_kinematics.inverse_kinematics(m_current_angles, destination_position);

  if (above_object_position.has_value() && above_object_grap_position.has_value() &&
      above_destination_position.has_value())
  {
    double gripper_angle = above_object_position.value()[0][0] + req.angle_difference;
    std::cout << "gripper angle: " << gripper_angle << std::endl;
    std::cout << "current angle: " << m_current_angles[0][0] << std::endl;
    moveGripper(0);
    moveArm(above_object_position, gripper_angle);
    moveArm(above_object_grap_position, gripper_angle);
    moveGripper(29);
    moveArm(above_destination_position, 0);
    moveGripper(0);

    robot_kinematica::al5dPositionGoal goal;
    goal.name = "PARK";
    goal.time = 2000;
    m_al5d_action_client.sendGoalAndWait(goal);
    res.finished = true;
  }
  else
  {
    res.finished = false;
  }

  return true;
}

RobotController::~RobotController()
{
}

void sendCoordinates()
{
  robot_kinematica::al5dPositionGoal goal;
}

void RobotController::moveGripper(const double degrees)
{
  robot_kinematica::al5dPositionGoal goal;
  goal.name = "moveGripper";
  goal.time = 2000;
  goal.degrees.push_back(degrees);
  goal.servos.push_back(GRIPPER);
  m_al5d_action_client.sendGoalAndWait(goal);
}
void RobotController::moveArm(const std::optional<Kinematics::Matrix<double, 4, 1>> position,
                              const double gripper_angle)
{
  auto coordinates = position.value();
  robot_kinematica::al5dPositionGoal goal;
  goal.name = "moveToObject";
  goal.time = 2000;
  goal.degrees.push_back(coordinates[0][0]);
  goal.degrees.push_back(coordinates[0][1]);
  goal.degrees.push_back(coordinates[0][2]);
  goal.degrees.push_back(coordinates[0][3] * -1);
  goal.degrees.push_back(gripper_angle);

  goal.servos.push_back(BASE);
  goal.servos.push_back(SHOULDER);
  goal.servos.push_back(ELBOW);
  goal.servos.push_back(WRIST);
  goal.servos.push_back(WRIST_ROTATE);
  m_al5d_action_client.sendGoalAndWait(goal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotController");

  RobotController robot_controller("robot_kinematica");

  ros::spin();
  return 0;
}
