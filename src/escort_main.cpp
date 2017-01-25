#include <ros/ros.h>
#include <ros/package.h>
#include "DefaultValues.h"
#include "DataStorage.h"
#include "SensorsModule.h"

ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
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
    if(DataStorage::GetInstance().Initialize(nodeHandlePrivate))
    {
        ROS_DEBUG("Data storage initialized successfully.");
    }
    else
    {
        ROS_ERROR("Failed to initialize data storage");
        return false;
    }
    if(SensorsModule::GetInstance().Initialize())
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
    DataStorage::GetInstance().UpdatePoseCooldowns(1.0f/mainLoopRate);
    SensorsModule::GetInstance().Update();
}

//TODO: usunąć
void Debug()
{
}

void Finish()
{
	delete nodeHandlePublic;
	delete nodeHandlePrivate;
    SensorsModule::GetInstance().Finish();
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
