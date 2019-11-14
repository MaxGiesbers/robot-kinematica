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
    std::cout << req.approx_0_x  << std::endl;
    std::cout << req.approx_0_y  << std::endl;
    std::cout << req.approx_1_x  << std::endl;
    std::cout << req.approx_1_y  << std::endl;
    std::cout << req.approx_3_x  << std::endl;

  double top = req.approx_3_x - req.approx_0_x;
  double side = req.approx_1_y - req.approx_0_y;
  double gripperAngle = 0;
  auto coordinates = position.value();
  double baseAngle = coordinates[0][0];  // De hoek waarin de base staat na berekening I.K.
    std::cout << baseAngle << std::endl;
  double xDev = req.approx_0_x - req.approx_1_x;  // Verschil tussen de X-coördinaten van linker- en rechterbovenhoek
  double yDev = req.approx_1_y - req.approx_0_y;  // Verschil tussen de Y-coördinaten van linker- en rechterbovenhoek

  double hyp = std::hypot(xDev, yDev);                // Berekening van hypotenusa
  double angle = std::asin(xDev / hyp) * 180 / M_PI;  // Berekening van de bovenste hoek die tussen de bovenzijde van
  if(req.origin_x < BASE_CARTESIAN_X)
  {
      angle = angle * -1;
  }

  std::cout <<"angle: " << angle << std::endl;                                                    // het object en een denkbeeldige loodrechte as wordt gevormd

  double angleDiff = correctForServoLimits(angle - baseAngle);  // Het verschil tussen wat de huidige rotatie is v.s.
    std::cout << "angle diff: " <<  angleDiff << std::endl; 
    // std::cout << "base Angle: " << baseAngle << std::endl;   
    // double difference = baseAngle - angleDiff ;
    // std::cout << "differnce: "   << difference << std::endl;  
    
    // std::cout << -1 - -1 << std::endl;                                             // wat deze moet zijn + correctie

    if(req.origin_x < BASE_CARTESIAN_X){
        gripperAngle = correctForServoLimits(angleDiff * -1);  // De hoek die de gripper moet aannemen om parrallel te
                                                                // staan met de zijden + correctie
    }
    else 
    {
        gripperAngle = correctForServoLimits(angleDiff);  // De hoek die de gripper moet aannemen om parrallel te
                                                                // staan met de zijden + correctie
    }
  

  if (top < side)
  {
    gripperAngle =
        correctForServoLimits(gripperAngle + 90);  // Correctie voor wanneer de boven-/onderkant groter zijn dan de
                                                   // zijden --> draait gripper een kwartslag extra + correctie
  }

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
    double gripper_angle = getGripperAngle(req, above_object_position);
    std::cout << gripper_angle << std::endl;
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
