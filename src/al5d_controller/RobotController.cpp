#include "al5d_controller/RobotController.h"
#include "matrix_class/Matrix.hpp"
#include "ros/ros.h"


namespace
{
const uint8_t MINIMAL_ARGUMENTS = 4;
const uint16_t QUEUE_SIZE = 1000;

}
RobotController::RobotController(const std::string& name, const std::string& positions_file_name,
                                       const std::string port): m_objectCoordinatesReceived(false)
{
    m_high_level_interface = std::make_shared<HighLevelInterface>(name, positions_file_name, port);
    m_subscriber = m_node_handle.subscribe("found_object", QUEUE_SIZE, &RobotController::callBack, this);
    moveObjectToDestination();

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


    m_objectCoordinatesReceived = true;
}

RobotController::~RobotController()
{

}

void RobotController::openGripper()
{
    //send coordinates...
}

void RobotController::moveObjectToDestination()
{

    //0.10, 0.10, 0.05

	const Kinematics::Matrix<double, 3, 1> destination = {{0.1},
	                                   {0.1},
	                                   {0.05}};

    
    Kinematics::Matrix<double, 4, 1> start = {{0},
	                                     {47},
	                                     {98},
	                                     {12.5}};


    Kinematics::Kinematics kinematics;
    std::optional<Kinematics::Matrix<double, 4, 1>> ik_solution = kinematics.inverse_kinematics(start, destination);
  
    auto testen = ik_solution.value();

    

    auto test = static_cast<int>(testen[0][0]);





    std::cout << testen[0][0] << std::endl;
    std::cout << testen[0][1] << std::endl;
    std::cout << testen[0][2] << std::endl;
    std::cout << testen[0][3] << std::endl;


    // while(m_objectCoordinatesReceived)
    // {


    // }
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
