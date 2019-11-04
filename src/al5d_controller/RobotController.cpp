#include "al5d_controller/RobotController.h"


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
    moveObjectToDestination();
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
    while(m_objectCoordinatesReceived)
    {


    }
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
