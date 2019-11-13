#include "al5d_controller/RobotController.h"
#include "matrix_class/Matrix.hpp"
#include "ros/ros.h"

namespace
{
const uint8_t MINIMAL_ARGUMENTS = 4;
const uint16_t QUEUE_SIZE = 1000;
}

RobotController::RobotController(const std::string& name): 
    m_objectCoordinatesReceived(false), 
    m_al5d_action_client(m_node_handle, name)
{
    m_subscriber = m_node_handle.subscribe("found_object", QUEUE_SIZE, &RobotController::callBack, this);
    ros::Duration(5).sleep();

    moveGripper(10);
}

void RobotController::callBack(const robot_kinematica::found_object& found_object)
{
    m_found_object.origin_x = found_object.origin_x;
    m_found_object.origin_y = found_object.origin_y;
    m_found_object.origin_z = found_object.origin_z;

    m_found_object.dimension_x = found_object.dimension_x; 
    m_found_object.dimension_y = found_object.dimension_y;
    m_found_object.dimension_z = found_object.dimension_z;

    m_found_object.destination_x = found_object.destination_x;
    m_found_object.destination_y = found_object.destination_y;
    m_found_object.destination_z = found_object.destination_z;
    
    ROS_INFO_STREAM("Received Found object");
    moveObjectToDestination(m_found_object);
    m_objectCoordinatesReceived = true;
}

RobotController::~RobotController()
{
}

void sendCoordinates()
{
    robot_kinematica::al5dPositionGoal goal;
}

void RobotController::moveObjectToDestination(const robot_kinematica::found_object& found_object)
{
    m_current_angles[0][0] = 0.0;
    m_current_angles[0][1] = -30.0;
    m_current_angles[0][2] = 115.0;
    m_current_angles[0][3] = -55.0;

    
    const Kinematics::Matrix<double, 3, 1> object_position(
    {{found_object.origin_y}, {found_object.origin_z}, {found_object.origin_x - 0.015}});

    const Kinematics::Matrix<double, 3, 1> object_position_below(
    {{found_object.origin_y}, {found_object.origin_z - 0.05}, {found_object.origin_x - 0.015}});
    
    //  const Kinematics::Matrix<double, 3, 1> destination_position(
    // {{found_object.destination_y}, {found_object.destination_z}, {found_object.destination_x - 0.015}});

    const Kinematics::Matrix<double, 3, 1> destination_position(
    {{found_object.destination_y}, {0}, {found_object.destination_x - 0.015}});

    auto above_object_position = m_kinematics.inverse_kinematics(m_current_angles, object_position);
    auto above_object_grap_position = m_kinematics.inverse_kinematics(m_current_angles, object_position_below);
    auto above_destination_position = m_kinematics.inverse_kinematics(m_current_angles, destination_position);

    moveGripper(0);
    moveArm(above_object_position);
    moveArm(above_object_grap_position);
    moveGripper(29);
    moveArm(above_destination_position);
    moveGripper(0);
    

    // auto above_shape_angles_opt =
	// 	Kinematics::inverse(above_shape_pos, 90.0, wrist_rotate);







    // robot_kinematica::al5dPositionGoal goal;

    // Kinematics::Matrix<double, 3, 1> destination = {{found_object.origin_y},
	//                                    {-0.02},
	//                                    {found_object.origin_x - 0.015}}; //Z and y are switched

    
    // Kinematics::Matrix<double, 4, 1> start = {{0},
	//                                      {-30},
	//                                      {115},
	//                                     {-55}};

    // std::optional<Kinematics::Matrix<double, 4, 1>> ik_solution = kinematics.inverse_kinematics(start, destination);

    // auto coordinates = ik_solution.value();

    // std::cout << coordinates[0][0] << "," << coordinates[0][1]  << "," << coordinates[0][2]  << "," << coordinates[0][3] << std::endl;

    // goal.name = "moveToObject";
    // goal.time = 2000;
    // goal.degrees.push_back(coordinates[0][0]);
    // goal.degrees.push_back(coordinates[0][1]    );
    // goal.degrees.push_back(coordinates[0][2]);
    // goal.degrees.push_back(coordinates[0][3] * -1);
    // //goal.degrees.push_back(-30);
  

    // goal.servos.push_back(BASE);
    // goal.servos.push_back(SHOULDER);
    // goal.servos.push_back(ELBOW);
    // goal.servos.push_back(WRIST);
    
    // m_al5d_action_client.sendGoal(goal);
    // ros::Duration(5).sleep();


    // // bool finished_before_timeout = m_al5d_action_client.waitForResult(ros::Duration(5.0));

    // // if (finished_before_timeout)
    // // {
    // //     std::cout << " finished" << std::endl;
    // // }

    // // std::cout << "komt hier" << std::endl;

    // ///////



    // goal.degrees.clear();


    // std::cout << "komt hier nu in" << std::endl;
    // destination = {{found_object.origin_y},
	//                                    {-0.07},
	//                                    {found_object.origin_x - 0.015}}; //Z and y are switched

    
    // start = {{coordinates[0][0]},
	//                                      {coordinates[0][1]},
	//                                      {coordinates[0][2]},
	//                                     {coordinates[0][3]}};


    // ik_solution = kinematics.inverse_kinematics(start, destination);
    // coordinates = ik_solution.value();


    // goal.name = "moveToObject";
    // goal.time = 2000;
    // goal.degrees.push_back(coordinates[0][0]);
    // goal.degrees.push_back(coordinates[0][1]);
    // goal.degrees.push_back(coordinates[0][2]);
    // goal.degrees.push_back(coordinates[0][3] * -1);
    // //goal.degrees.push_back(-30);
  

    // goal.servos.push_back(BASE);
    // goal.servos.push_back(SHOULDER);
    // goal.servos.push_back(ELBOW);
    // goal.servos.push_back(WRIST);

    // m_al5d_action_client.sendGoal(goal);
    // ros::Duration(5).sleep();

    // robot_kinematica::al5dPositionGoal goal2;

    // // goal.degrees.clear();
    // // goal.name.clear();
    // goal2.name = "CLOSE_GRIPPER";
    // goal2.degrees.push_back(28);

    // goal2.servos.push_back(5);
    // goal2.time = 2000;

    // m_al5d_action_client.sendGoal(goal2);
    // ros::Duration(7).sleep();

    // goal2.degrees.clear();
    // goal2.name.clear();
    // goal2.name = "PARK";
    // m_al5d_action_client.sendGoal(goal2);




    // m_high_level_interface->park();


    // m_al5d_action_client.sendGoalAndWait(goal);

}


void RobotController::moveGripper(const double degrees)
{
    robot_kinematica::al5dPositionGoal goal;
    goal.name = "moveGripper";
    goal.time = 2000;
    goal.degrees.push_back(degrees);
    goal.servos.push_back(GRIPPER);
    m_al5d_action_client.sendGoal(goal);
    ros::Duration(3).sleep();
}
void RobotController::moveArm(const std::optional<Kinematics::Matrix<double, 4, 1>>  position)
{
    auto coordinates = position.value();
    robot_kinematica::al5dPositionGoal goal;
    goal.name = "moveToObject";
    goal.time = 2000;
    goal.degrees.push_back(coordinates[0][0]);
    goal.degrees.push_back(coordinates[0][1]);
    goal.degrees.push_back(coordinates[0][2]);
    goal.degrees.push_back(coordinates[0][3] * -1);

    goal.servos.push_back(BASE);
    goal.servos.push_back(SHOULDER);
    goal.servos.push_back(ELBOW);
    goal.servos.push_back(WRIST);
    m_al5d_action_client.sendGoal(goal);
    ros::Duration(3).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotController");

  RobotController robot_controller("robot_kinematica");

  ros::spin();
  return 0;
}
