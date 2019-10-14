#ifndef HIGHLEVELINTERFACE_H_
#define HIGHLEVELINTERFACE_H_

/**
 * @brief The High Level Interface that receives a sequence of motions of the motion planner to execute and passes it on to the low level driver.
 * 
 * @file HighLevelInterface.h
 * @author Joris van Zeeland, Max Giesbers
 * @date 2018-09-25
 */

//Ros includes
#include <ros/ros.h>
#include "al5d_controller/al5dPositionAction.h"
#include <actionlib/server/simple_action_server.h>
#include "al5d_controller/eStop.h"

//other includes
#include "../LowLevel/SerialCommunication.h"
#include "../LowLevel/Servo.h"
#include "../LowLevel/LowLevelDriver.h"

#include "Position.h"
#include <iostream>
#include <vector>
#include <thread>
#include <sstream>
namespace Highlevel{
class HighLevelInterface
{
  public:
  /**
   * @brief Construct a new High Level Interface object
   * 
   * @param name contains a string with the the name of the topic.
   * @param aSerial contains a SerialCommunication object for communicating between
   * the SCC-32u board and the low level driver.
   */
    HighLevelInterface(const std::string& name , const std::string& positionsFileName, const std::string port);
    /**
     * @brief Destroy the High Level Interface object
     * 
     */
    ~HighLevelInterface(void);
    /**
     * @brief Callback function which automatically asynchronously executes the Action file. 
     * 
     * @param goal contains a Al5dPositionGoal const pointer with messages for communicating between
     * high level interface and the motionplanner. 
     * for communicating between 
     */
    void executeCB(const al5d_controller::al5dPositionGoalConstPtr &goal);
    /**
     * @brief concatMessage will concat the message in the right format before it sends
     * the message to the low level driver.
     * 
     * @param goal contains a Al5dPositionGoal const pointer with messages for communicating between
     * high level interface and the motionplanner. 
     */
    void concatMessage(const al5d_controller::al5dPositionGoalConstPtr &goal);
    /**
     * @brief concatMessage will concat the message in the right format before it sends
     * the message to the low level driver.
     * 
     * @param goal contains a Al5dPositionGoal const pointer with messages for communicating between
     * high level interface and the motionplanner. 
     */
    void concatMessage(const Position& position);
    /**
     * @brief for initializing all the servo objects of the robot arm.
     */
    void initServoList();
    /**
     * @brief a service that will immediately stop the robot arm from moving after the timer has reached his time limit.
     * 
     * @param req contains the estop request for stopping the robot arm.
     * @param res Contains the response when the robot arm has stopped moving. 
     * @return true When the robot arm has stopped.
     * @return false When a exception will throw up.
     */
    bool emergencyStop(al5d_controller::eStop::Request& req, al5d_controller::eStop::Response& res);
    /**
     * @brief Parses the pre-programmed positions from a csv file into Position objects
     * 
     * @param fileName the name of the .csv file that has to be parsed
     * @return true if parsing succeeded
     * @return false if parsing failed
     */
    bool parseProgrammedPositions(const std::string& fileName);
    /**
     * @brief This runs the state machine that checks the incoming goals and handles based on the name of the goal.
     * 
     * @param goal The incoming ActionGoal from the motion planner.
     */
    void run(const al5d_controller::al5dPositionGoalConstPtr &goal);


  private:
    ros::NodeHandle nh_;    
    actionlib::SimpleActionServer<al5d_controller::al5dPositionAction> al5dActionServer;
    std::string name;
    Lowlevel::LowLevelDriver lowLevelComponent;
    std::vector<Lowlevel::Servo> servoList;
    std::vector<Position> positionList;
};
}
#endif // HIGHLEVELINTERFACE_H_
