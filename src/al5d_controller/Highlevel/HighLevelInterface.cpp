#include "al5d_controller/Highlevel/HighLevelInterface.h"

#include <fstream>
namespace Highlevel{
HighLevelInterface::HighLevelInterface(const std::string& name, const std::string& positionsFileName, const std::string port) : 
                al5dActionServer(nh_, name, boost::bind(&HighLevelInterface::executeCB, this, _1), 
                    false),name(name),lowLevelComponent(port)
{
    initServoList();
    al5dActionServer.start();
    std::cout << positionsFileName << std::endl;
    parseProgrammedPositions(positionsFileName);
}

HighLevelInterface::~HighLevelInterface(void)
{
}

void HighLevelInterface::executeCB(const al5d_controller::al5dPositionGoalConstPtr &goal)
{
    ROS_INFO("STATE: HANDLE GOAL: %s", (*goal).name.c_str());
    if(!ros::ok())
    {
        al5dActionServer.setAborted();
        ROS_INFO("%s Shutting down",name.c_str());
    } 
    else 
    {
        run(goal);
    }
}


void HighLevelInterface::run(const al5d_controller::al5dPositionGoalConstPtr &goal)
{
    if(goal->name.compare(std::string{"PARK"}) == 0)
    {
        auto findFunction = [](const Position& p){return p.getName().compare("PARK") == 0;};
        auto result = std::find_if(positionList.begin(), positionList.end(), findFunction);
        concatMessage((*result));
    } 
    else if (goal->name.compare(std::string{"STRAIGHT"}) == 0)
    {
        auto findFunction = [](const Position& p){return p.getName().compare("STRAIGHT") == 0 ;};
        auto result = std::find_if(positionList.begin(), positionList.end(), findFunction);
        concatMessage((*result));
    }
    else if (goal->name.compare("READY") == 0)
    {
        auto findFunction = [](const Position& p){return p.getName().compare("READY") == 0;};
        auto result = std::find_if(positionList.begin(), positionList.end(), findFunction);
        concatMessage((*result));
    }
    else 
    {
        concatMessage(goal);
    }
} 

bool HighLevelInterface::emergencyStop(al5d_controller::eStop::Request& req, al5d_controller::eStop::Response& res)
{
    lowLevelComponent.emergencyStop();
    return true;
}  

void HighLevelInterface::concatMessage(const al5d_controller::al5dPositionGoalConstPtr &goal)
{
    std::vector <int> positions;
    for(int i =0; i < (*goal).degrees.size(); ++i){
        int pw = servoList.at(i).degreesToPW((*goal).degrees[i]);
        positions.push_back(pw);
    }
    lowLevelComponent.goToPosition(positions,(*goal).time);

    ros::Duration(4).sleep();
    al5dActionServer.setSucceeded();
    ROS_INFO_STREAM("STATE: SUCCEEDED: " << (*goal).name);

}

void HighLevelInterface::concatMessage(const Position& position) 
{
    std::vector <int> positions;

    for(int i =0; i < position.getDegrees().size(); ++i)
    {
        int pw = servoList.at(i).degreesToPW(position.getDegrees()[i]);
        positions.push_back(pw);
    }
    lowLevelComponent.goToPosition(positions,position.getTime());

    ros::Duration(4).sleep();
    al5dActionServer.setSucceeded();
    ROS_INFO_STREAM("STATE: SUCCEEDED: " << position.getName());  
}

void HighLevelInterface::initServoList()
{
    servoList.push_back(Lowlevel::Servo("Base", -90, 90, 500, 2500));
    servoList.push_back(Lowlevel::Servo("Shoulder", -30, 90, 1100, 2500));
    servoList.push_back(Lowlevel::Servo("Elbow", 0, 135, 650, 2500));
    servoList.push_back(Lowlevel::Servo("Wrist", -90, 90, 500, 2500));
    servoList.push_back(Lowlevel::Servo("WristRotate", -90, 90, 500, 2500));
    servoList.push_back(Lowlevel::Servo("Gripper", -90, 90, 500, 2500));
}

bool HighLevelInterface::parseProgrammedPositions(const std::string& fileName)
{
    std::ifstream file(fileName);
	if(!file.is_open())
	{
		std::cout << "There are no preprogrammed positions in " << fileName << std::endl;
		return false;
	}
	std::string value;
	char spacer=',';

	// Get rid of the first line with information headers
	std::getline(file,value,'\n');

	while(file.good())
	{
        //init all variables from file
        std::string name = "";
        int degreesBase,
            degreesShoulder,
            degreesElbow,
            degreesWrist,
            degreesWristRotate,
            degreesGripper,
            time = 0;
		
        //name
        std::getline( file, value, spacer );
		name=value;
        //DEGREESBASE
		std::getline ( file, value, spacer );
		degreesBase=std::stoi(value);
        //DEGREESSHOULDER
		std::getline ( file, value, spacer );
		degreesShoulder=std::stoi(value);
         //DEGREESELBOW
		std::getline ( file, value, spacer );
		degreesElbow=std::stoi(value);
        //DEGREESWRIST
		std::getline ( file, value, spacer );
		degreesWrist=std::stoi(value);
        //DEGREESWRISTROTATE
		std::getline ( file, value, spacer );
		degreesWristRotate=std::stoi(value);
        //DEGREESGRIPPER
		std::getline ( file, value, spacer );
		degreesGripper=std::stoi(value);
        //TIME
		std::getline ( file, value, spacer );
		time=std::stoi(value);
        //END OF LINE
		std::getline(file,value,'\n');

		//put the variables in list of Positions
        Position position;

        position.setName(name);
        position.addDegrees(degreesBase); 
        position.addDegrees(degreesShoulder); 
        position.addDegrees(degreesElbow); 
        position.addDegrees(degreesWrist); 
        position.addDegrees(degreesWristRotate); 
        position.addDegrees(degreesGripper);
        position.setTime(time);
        positionList.push_back(position);
	}
	file.close();
	return true;
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "al5d_interface");
    //std::cout << argv[2] << std::endl;
    std::cout << argc << std::endl;
    if(argc != 4)
    {
        std::cout << "Missing arguments. Try something like: rosrun al5d_controller al5d_interface ProgrammedPositions.csv /dev/ttyUSB0" << std::endl;
        return 1;
    }
    Highlevel::HighLevelInterface highLevelInterface("al5d_controller", argv[2], argv[3]);
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("eStop", &Highlevel::HighLevelInterface::emergencyStop, &highLevelInterface);
    ros::spin();
    return 0;
}


