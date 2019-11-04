#include "robot_kinematica/found_object.h"
#include <ros/ros.h>
#include <memory>
#include "al5d_controller/Highlevel/HighLevelInterface.h"
#include "al5d_controller/Kinematics.h"

class RobotController
{
    public:
    RobotController(const std::string& name, const std::string& positions_file_name, const std::string port);
    ~RobotController();

    private:
    void callBack(const robot_kinematica::found_object& found_object);
    void openGripper();
    ros::Subscriber m_subscriber;
   
    std::shared_ptr<HighLevelInterface> m_high_level_interface;
    ros::NodeHandle m_node_handle;
    void moveObjectToDestination();
    bool m_objectCoordinatesReceived;
    robot_kinematica::found_object m_found_object;

};