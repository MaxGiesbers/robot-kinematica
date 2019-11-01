#include <ros/ros.h>
#include "robot_kinematica/al5dPositionAction.h"
#include <actionlib/client/simple_action_client.h>
#include "robot_kinematica/eStop.h"

#include <iostream>
#include <queue>
#include <fstream>

typedef ::actionlib::SimpleActionClient<robot_kinematica::al5dPositionAction> MotionPlanner;
std::queue<robot_kinematica::al5dPositionGoal> goals;

bool parseCSVFile(std::string fileName)
{
  std::ifstream file(fileName);
  if (!file.is_open())
  {
    std::cout << "There is no configuration in " << fileName << std::endl;
    return false;
  }
  std::string value = "";

  // THE CHAR BETWEEN 2 VALUES
  char spacer = ',';

  // GET RID OF THE FIRST LINE WITH ALL THE INFORMATIONHEADERS
  std::getline(file, value, '\n');

  // WHILE THE FILE HAS LINES LEFT TO READ
  while (file.good())
  {
    robot_kinematica::al5dPositionGoal goal;
    // INITIALISE ALL THE VALUES FROM THE FILE
    std::string name;
    int degreesBase, degreesShoulder, degreesElbow, degreesWrist, degreesWristRotate, degreesGripper, time = 0;

    // NAME
    std::getline(file, value, spacer);
    name = value;

    if ((name.compare(std::string{ "STRAIGHT" }) != 0) && (name.compare(std::string{ "READY" }) != 0) &&
        (name.compare(std::string{ "PARK" })))
    {
      // DEGREESBASE
      std::getline(file, value, spacer);
      degreesBase = std::stoi(value);
      // DEGREESSHOULDER
      std::getline(file, value, spacer);
      degreesShoulder = std::stoi(value);
      // DEGREESELBOW
      std::getline(file, value, spacer);
      degreesElbow = std::stoi(value);
      // DEGREESWRIST
      std::getline(file, value, spacer);
      degreesWrist = std::stoi(value);
      // DEGREESWRISTROTATE
      std::getline(file, value, spacer);
      degreesWristRotate = std::stoi(value);
      // DEGREESGRIPPER
      std::getline(file, value, spacer);
      degreesGripper = std::stoi(value);
      // TIME
      std::getline(file, value, spacer);
      goal.time = std::stoi(value);
      goal.degrees.push_back(degreesBase);
      goal.degrees.push_back(degreesShoulder);
      goal.degrees.push_back(degreesElbow);
      goal.degrees.push_back(degreesWrist);
      goal.degrees.push_back(degreesWristRotate);
      goal.degrees.push_back(degreesGripper);
    }
    // END OF LINE
    std::getline(file, value, '\n');
    // PUTS THE VARIABLES IN THE LIST OF GOALS
    goal.name = name;
    goals.push(goal);
  }

  file.close();

  return true;
}

void motionPlanner(MotionPlanner& ac)
{
  robot_kinematica::al5dPositionGoal goal = goals.front();
  goals.pop();
  ROS_INFO("Send Goal: %s", goal.name.c_str());
  ac.sendGoalAndWait(goal, ros::Duration(2, 300), ros::Duration(5, 0));

  std::stringstream ss;
  for (auto degree : goal.degrees)
  {
    ss << degree;
  }

  ROS_DEBUG_STREAM("EVENT: SEND GOAL: " << goal.name << " degrees: " << ss.str());
}

void emergencyCallback(const ros::TimerEvent& event)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robot_kinematica::eStop>("eStop");
  robot_kinematica::eStop eStopSrv;
  eStopSrv.request.stop = true;
  if (client.call(eStopSrv))
  {
    ROS_DEBUG_STREAM("EVENT:EMERGENCY STOP: Emergency stop service call succeeded");
  }
  else
  {
    ROS_DEBUG_STREAM("EVENT:EMERGENCY STOP: Emergency stop service call failed");
  }
  std::queue<robot_kinematica::al5dPositionGoal> empty;
  std::swap(goals, empty);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "al5d_motionplanner");
  MotionPlanner ac("robot_kinematica", true);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  try
  {
    if (!parseCSVFile(argv[1]))
      return 0;
  }
  catch (std::exception& e)
  {
    std::cout << "Name or location of the config file is incorrect." << std::endl;
    exit(EXIT_FAILURE);
  }
  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the servo server to come up");
  }
  ros::NodeHandle nh;
  ros::Timer timer = nh.createTimer(ros::Duration(22), emergencyCallback);
  ROS_INFO("Al5d server connected, starting motion planner...");
  motionPlanner(ac);

  while (goals.empty() == false)
  {
    ac.waitForResult(ros::Duration(5.0));
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      motionPlanner(ac);
    }
  }
  return 0;
}
