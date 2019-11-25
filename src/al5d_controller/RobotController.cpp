#include "al5d_controller/RobotController.h"
#include "matrix_class/Matrix.hpp"
#include "ros/ros.h"

namespace
{
const uint8_t MINIMAL_ARGUMENTS = 4;
const uint16_t QUEUE_SIZE = 1000;
const double BASE_CARTESIAN_X = 343;
}  // namespace

RobotController::RobotController(const std::string& name)
  : m_objectCoordinatesReceived(false), m_al5d_action_client(m_node_handle, name)
{
  m_service = m_node_handle.advertiseService("found_object", &RobotController::moveObjectToDestination, this);
}

double RobotController::getGripperAngle(robot_kinematica::found_object::Request& req,
                                        const std::optional<Kinematics::Matrix<double, 4, 1>> position)
{
  const double a = (req.approx_3_x - req.approx_0_x) / 2;
  const double b = req.approx_0_x + a;
  const double c = (req.approx_3_y - req.approx_0_y) / 2;
  const double d = req.approx_0_y + c;
  double e = 0;
  double f = 0;

  double baseAngle = position.value()[0][0];
  const double side_upper = std::hypot(req.approx_3_x - req.approx_0_x, req.approx_3_y - req.approx_0_y);
  const double side_left = std::hypot(req.approx_0_x - req.approx_1_x, req.approx_1_y - req.approx_0_y);

  if (b < req.origin_cartesian_x)
  {
    e = std::hypot(req.origin_cartesian_x - b, req.origin_cartesian_y - d);
    f = std::asin((req.origin_cartesian_x - b) / e) * 180 / M_PI * -1;
  }
  else
  {
    e = std::hypot(b - req.origin_cartesian_x, req.origin_cartesian_y - d);
    f = std::asin((b - req.origin_cartesian_x) / e) * 180 / M_PI;
  }

  double diff = correctForServoLimits(baseAngle - f);
  double gripperAngle = 0 + diff;

  if (side_upper > side_left)
  {
    gripperAngle = correctForServoLimits(gripperAngle + 90);
  }

  std::cout << "SideUpper: " << side_upper << std::endl;
  std::cout << "SideLeft: " << side_left << std::endl;
  std::cout << "F: " << f << std::endl;
  std::cout << "diff: " << diff << std::endl;
  std::cout << "BaseAngle: " << baseAngle << std::endl;
  std::cout << "GripperAngle: " << gripperAngle << std::endl;

  return gripperAngle;
}

double RobotController::correctForServoLimits(double angle)
{
  if (angle < -90)
  {
    angle += 180;
  }
  else if (angle > 90)
  {
    angle -= 180;
  }

  return angle;
}

bool RobotController::moveObjectToDestination(robot_kinematica::found_object::Request& req,
                                              robot_kinematica::found_object::Response& res)
{
  m_current_angles[0][0] = 0.0;
  m_current_angles[0][1] = -30.0;
  m_current_angles[0][2] = 115.0;
  m_current_angles[0][3] = -55.0;

  const Kinematics::Matrix<double, 3, 1> object_position({ { req.origin_y }, { 0 }, { req.origin_x } });

  const Kinematics::Matrix<double, 3, 1> object_position_below({ { req.origin_y }, { -0.05 }, { req.origin_x } });

  const Kinematics::Matrix<double, 3, 1> destination_position(
      { { req.destination_y }, { 0.02 }, { req.destination_x } });

  const Kinematics::Matrix<double, 3, 1> destination_position_below(
      { { req.destination_y }, { -0.03 }, { req.destination_x } });

  auto above_object_position = m_kinematics.inverse_kinematics(m_current_angles, object_position);
  auto above_object_grap_position = m_kinematics.inverse_kinematics(m_current_angles, object_position_below);
  auto above_destination_position = m_kinematics.inverse_kinematics(m_current_angles, destination_position);
  auto above_destination_drop_position = m_kinematics.inverse_kinematics(m_current_angles, destination_position_below);

  if (above_object_position.has_value() && above_object_grap_position.has_value() &&
      above_destination_position.has_value())
  {
    double gripper_angle = 0;

    if (!req.approx_0_x == 0 && !req.approx_0_y == 0 && !req.approx_1_x == 0 && !req.approx_1_y == 0 &&
        !req.approx_3_x == 0)
    {
      gripper_angle = getGripperAngle(req, above_object_position);
    }

    moveGripper(0);
    moveArm(above_object_position, gripper_angle);
    moveArm(above_object_grap_position, gripper_angle);
    moveGripper(29);
    moveArm(above_object_position, gripper_angle);
    moveArm(above_destination_position, 0);
    moveArm(above_destination_drop_position, 0);
    moveGripper(0);
    moveArm(above_destination_position, 0);

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
  try
  {
    RobotController robot_controller("robot_kinematica");
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
  ros::spin();

  return 0;
}
