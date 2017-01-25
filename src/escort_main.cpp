#include <ros/ros.h>
#include <ros/package.h>
#include "DefaultValues.h"
#include "DataStorage.h"
#include "SensorsModule.h"

ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
DataStorage* dataStorage;
SensorsModule* sensorsModule;
double mainLoopRate;

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
    if(dataStorage->Initialize(nodeHandlePrivate))
    {
        ROS_DEBUG("Data storage initialized successfully.");
    }
    else
    {
        ROS_ERROR("Failed to initialize data storage");
        return false;
    }
    sensorsModule = new SensorsModule(dataStorage);
    if(sensorsModule->Initialize())
    {
        ROS_DEBUG("Sensors module initialized successfully.");
    }
    else
    {
        ROS_ERROR("Failed to initialize sensors module.");
        return false;
    }
	return true;
}

void Update()
{
    dataStorage->UpdatePoseCooldowns(1.0f/mainLoopRate);
    sensorsModule->Update();
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
    sensorsModule->Finish();
    delete sensorsModule;
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
            Update();
            Debug();
			mainLoopRate.sleep();
		}
	}
	Finish();
	return 0;
}
