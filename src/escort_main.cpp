#include <ros/ros.h>
#include <ros/package.h>
#include "DefaultValues.h"
#include "DataStorage.h"

ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
double mainLoopRate;
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
    if(!dataStorage->Initialize(nodeHandlePrivate))
    {
        return false;
    }

    //OpenNI initialization
    //...

	return true;
}

void UpdateSensorsData()
{
    dataStorage->UpdatePoseCooldowns(1.0f/mainLoopRate);
}

//TODO: usunąć
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
        ROS_INFO("Initialization complete, starting program.");
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
