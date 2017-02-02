#include <ros/ros.h>
#include <ros/package.h>
#include "Common.h"
#include "SensorsModule.h"
#include "TaskModule.h"
#include "MobilityModule.h"
#include "DataStorage.h"

#define DEFAULT_ESCORT_MAIN_LOG_LEVEL Debug
#define DEFAULT_MAIN_LOOP_RATE 30

LogLevels logLevel;
ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
double mainLoopRate;
double mainLoopTime;

bool Initialization()
{
	nodeHandlePublic = new ros::NodeHandle();
	nodeHandlePrivate = new ros::NodeHandle("~");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    int _logLevel;
    if(!nodeHandlePrivate->getParam("escortMainLogLevel", _logLevel))
    {
        ROS_WARN("Log level not found, using default");
        logLevel = DEFAULT_ESCORT_MAIN_LOG_LEVEL;
    }
    else
    {
        switch (_logLevel)
        {
            case 0:
                logLevel = Debug;
                break;
            case 1:
                logLevel = Info;
                break;
            case 2:
                logLevel = Warn;
                break;
            case 3:
                logLevel = Error;
                break;
            default:
                ROS_WARN("Requested invalid log level, using default");
                logLevel = DEFAULT_ESCORT_MAIN_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("mainLoopRate", mainLoopRate))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of mainLoopRate not found, using default: %d.", DEFAULT_MAIN_LOOP_RATE);
        }
        mainLoopRate = DEFAULT_MAIN_LOOP_RATE;
    }
    mainLoopTime = 1/mainLoopRate;
    if(DataStorage::GetInstance().Initialize(nodeHandlePrivate))
    {
        if(logLevel <= Debug)
        {
            ROS_DEBUG("Data storage initialized successfully.");
        }
    }
    else
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Failed to initialize data storage");
        }
        return false;
    }
    if(SensorsModule::GetInstance().Initialize(nodeHandlePrivate))
    {
        if(logLevel <= Debug)
        {
            ROS_DEBUG("Sensors module initialized successfully.");
        }
    }
    else
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Failed to initialize sensors module.");
        }
        return false;
    }
    //TODO: inicjalizacja identyfikacji
    if(MobilityModule::GetInstance().Initialize(nodeHandlePublic, nodeHandlePrivate))
    {
        if(logLevel <= Debug)
        {
            ROS_DEBUG("Mobility module initialized successfully.");
        }
    }
    else
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Failed to initialize mobility module.");
        }
        return false;
    }
    if(TaskModule::GetInstance().Initialize(nodeHandlePrivate))
    {
        if(logLevel <= Debug)
        {
            ROS_DEBUG("Task module initialized successfully.");
        }
    }
    else
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Failed to initialize task module.");
        }
        return false;
    }
	return true;
}

void Update()
{
    //TODO: topic module update
    SensorsModule::GetInstance().Update();
    //TODO: identification module update
    TaskModule::GetInstance().Update();
    MobilityModule::GetInstance().Update();
    DataStorage::GetInstance().Update(mainLoopTime);
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
        if(logLevel <= Info)
        {
            ROS_INFO("Initialization complete, starting program.");
        }
		ros::Rate mainLoopRate(mainLoopRate);
		while (ros::ok())
		{
            Update();
			mainLoopRate.sleep();
		}
	}
	Finish();
	return 0;
}
