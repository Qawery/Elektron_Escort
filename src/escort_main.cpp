#include <ros/ros.h>
#include <ros/package.h>

//Default parameters values
#define DEFAULT_MAIN_LOOP_RATE 30

ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
float mainLoopRate;

bool Initialization()
{
	nodeHandlePublic = new ros::NodeHandle();
	nodeHandlePrivate = new ros::NodeHandle("~");
    if(!nodeHandlePrivate->getParam("MainLoopRate", mainLoopRate))
    {
        ROS_WARN("Value of mainLoopRate not found, using default.");
        mainLoopRate = DEFAULT_MAIN_LOOP_RATE;
    }
	return true;
}

void UpdateSensorsData()
{

}

void Finish()
{
	delete nodeHandlePublic;
	delete nodeHandlePrivate;
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
			mainLoopRate.sleep();
		}
	}
	Finish();
	return 0;
}
