#include "al5d_controller/RobotController.h"
#include "matrix_class/Matrix.hpp"
#include "ros/ros.h"

namespace
{
const uint8_t MINIMAL_ARGUMENTS = 4;
const uint16_t QUEUE_SIZE = 1000;
}

RobotController::RobotController(const std::string& name, const std::string& positions_file_name,
                                       const std::string port): 
    m_objectCoordinatesReceived(false), 
    m_al5d_action_client(m_node_handle, name)
{
    m_high_level_interface = std::make_shared<HighLevelInterface>(name, positions_file_name, port);
    m_subscriber = m_node_handle.subscribe("found_object", QUEUE_SIZE, &RobotController::callBack, this);
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
    const int8_t BASE = 0;
    const int8_t SHOULDER = 1;
    const int8_t ELBOW = 2;
    const int8_t WRIST = 3;

    Kinematics::Kinematics kinematics;
    robot_kinematica::al5dPositionGoal goal;

    Kinematics::Matrix<double, 3, 1> destination = {{found_object.origin_y},
	                                   {-0.02},
	                                   {found_object.origin_x - 0.015}}; //Z and y are switched

    
    Kinematics::Matrix<double, 4, 1> start = {{0},
	                                     {-30},
	                                     {115},
	                                    {-55}};

    std::optional<Kinematics::Matrix<double, 4, 1>> ik_solution = kinematics.inverse_kinematics(start, destination);

    auto coordinates = ik_solution.value();

    std::cout << coordinates[0][0] << "," << coordinates[0][1]  << "," << coordinates[0][2]  << "," << coordinates[0][3] << std::endl;

    goal.name = "moveToObject";
    goal.time = 2000;
    goal.degrees.push_back(coordinates[0][0]);
    goal.degrees.push_back(coordinates[0][1]    );
    goal.degrees.push_back(coordinates[0][2]);
    goal.degrees.push_back(coordinates[0][3] * -1);
    //goal.degrees.push_back(-30);
  

    goal.servos.push_back(BASE);
    goal.servos.push_back(SHOULDER);
    goal.servos.push_back(ELBOW);
    goal.servos.push_back(WRIST);
    m_al5d_action_client.sendGoal(goal);

    bool finished_before_timeout = m_al5d_action_client.waitForResult(ros::Duration(5.0));

    if (finished_before_timeout)
    {
        std::cout << " finished" << std::endl;
    }

    std::cout << "komt hier" << std::endl;

    ///////


    // ros::Duration(5).sleep();

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
    // m_high_level_interface->closeGripper();
    // // ros::Duration(3).sleep();
    // m_high_level_interface->park();


    // m_al5d_action_client.sendGoalAndWait(goal);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RobotControler");
  if (argc != MINIMAL_ARGUMENTS)
  {
    ROS_WARN_STREAM("Missing arguments. Try something like: rosrun robot_kinematica al5d_interface "
                    "ProgrammedPositions.csv /dev/ttyUSB0");
    return 1;
  }

  RobotController robot_controller("robot_kinematica", argv[2], argv[3]);

  ros::spin();
  return 0;
}
