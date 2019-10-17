#ifndef HIGHLEVELINTERFACE_H_
#define HIGHLEVELINTERFACE_H

#include "al5d_controller/al5dPositionAction.h"
#include <actionlib/server/simple_action_server.h>
#include "al5d_controller/eStop.h"
#include "../LowLevel/LowLevelDriver.h"

#include "Position.h"
#include <vector>

class HighLevelInterface
{
  public:
    HighLevelInterface(const std::string& name , const std::string& positions_file_name, const std::string port);
    ~HighLevelInterface(void);
    void executeCB(const al5d_controller::al5dPositionGoalConstPtr &goal);
    void concatMessage(const al5d_controller::al5dPositionGoalConstPtr &goal);
    void concatMessage(const Position& position);
    void initServoList();
    bool emergencyStop(al5d_controller::eStop::Request& req, al5d_controller::eStop::Response& res);
    bool parseProgrammedPositions(const std::string& fileName);
    void run(const al5d_controller::al5dPositionGoalConstPtr &goal);


  private:
    ros::NodeHandle m_node_handle;    
    actionlib::SimpleActionServer<al5d_controller::al5dPositionAction> m_al5d_action_server;
    std::string m_name;
    LowLevelDriver m_low_level_component;
    std::vector<Servo> m_servo_list;
    std::vector<Position> m_position_list;
};

#endif // HIGHLEVELINTERFACE_H_
