#include <ros/ros.h>
#include <ros/package.h>
#include "DataStorage.h"

//Default parameters values
#define DEFAULT_MAIN_LOOP_RATE 30
#define DEFAULT_MAX_USERS 10
#define DEFAULT_POSE_COOLDOWN_TIME 3
double mainLoopRate;

ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
DataStorage* dataStorage;

bool Initialization()
{
	//ROS and program initialization
	nodeHandlePublic = new ros::NodeHandle();
	nodeHandlePrivate = new ros::NodeHandle("~");
    if(!nodeHandlePrivate->getParam("mainLoopRate", mainLoopRate))
    {
        ROS_WARN("Value of mainLoopRate not found, using default: %d.", DEFAULT_MAIN_LOOP_RATE);
        mainLoopRate = DEFAULT_MAIN_LOOP_RATE;
    }
    dataStorage = new DataStorage();
    int maxUsers;
	if(!nodeHandlePrivate->getParam("maxUsers", maxUsers))
	{
		ROS_WARN("Value of maxUsers not found, using default: %d.", DEFAULT_MAX_USERS);
		maxUsers = DEFAULT_MAX_USERS;
	}
    dataStorage->SetMaxUsers(maxUsers);
    double poseCooldownTime;
    if(!nodeHandlePrivate->getParam("poseCooldownTime", poseCooldownTime))
    {
        ROS_WARN("Value of poseCooldownTime not found, using default: %d.", DEFAULT_POSE_COOLDOWN_TIME);
        poseCooldownTime = DEFAULT_POSE_COOLDOWN_TIME;
    }
    dataStorage->SetPoseCooldownTime(poseCooldownTime);

    //OpenNI initialization

	return true;
}

void UpdateSensorsData()
{
    dataStorage->UpdatePoseCooldowns(1.0f/mainLoopRate);
}

void Debug()
{
}

void Finish()
{
	delete nodeHandlePublic;
	delete nodeHandlePrivate;
    delete dataStorage;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "escort_main");
	if(Initialization())
	{
		ros::Rate mainLoopRate(mainLoopRate);
		while (ros::ok())
		{
            UpdateSensorsData();
            Debug();
			mainLoopRate.sleep();
		}
	}
	Finish();
	return 0;
}
