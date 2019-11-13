#include "robot_kinematica/found_object.h"
#include <ros/ros.h>
#include <memory>
#include "al5d_controller/Highlevel/HighLevelInterface.h"
#include <actionlib/client/simple_action_client.h>
#include "al5d_controller/Kinematics.h"

class RobotController
{
    public:
    RobotController(const std::string& name, const std::string& positions_file_name, const std::string port);
    ~RobotController();

    private:
    void callBack(const robot_kinematica::found_object& found_object);
    ros::Subscriber m_subscriber;
   
    std::shared_ptr<HighLevelInterface> m_high_level_interface;
    ros::NodeHandle m_node_handle;
    void moveObjectToDestination(const robot_kinematica::found_object& found_object);
    bool m_objectCoordinatesReceived;
    robot_kinematica::found_object m_found_object;
    actionlib::SimpleActionClient<robot_kinematica::al5dPositionAction> m_al5d_action_client;
};