#define DEFAULT_ESCORT_MAIN_LOG_LEVEL Info
#define DEFAULT_MAIN_LOOP_RATE 30.0

#include <ros/ros.h>
#include <ros/package.h>
#include "Common.h"
//TODO: topic include
#include "Modules/SensorsModule.h"
#include "Modules/IdentificationModule.h"
#include "Modules/TaskModule.h"
#include "Modules/MobilityModule.h"
#include "Modules/DataStorage.h"


ros::NodeHandle* nodeHandlePublic;
ros::NodeHandle* nodeHandlePrivate;
LogLevels logLevel;
double mainLoopRate;
double mainLoopTime;


bool Initialization() {
    if(logLevel <= Info) {
        ROS_INFO("EscortMain: Initialization start");
    }
	nodeHandlePublic = new ros::NodeHandle();
	nodeHandlePrivate = new ros::NodeHandle("~");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    int _logLevel;
    if(!nodeHandlePrivate->getParam("escortMainLogLevel", _logLevel)) {
        ROS_WARN("EscortMain: Log level not found, using default");
        logLevel = DEFAULT_ESCORT_MAIN_LOG_LEVEL;
    }
    else {
        switch (_logLevel) {
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
                ROS_WARN("EscortMain: Requested invalid log level, using default");
                logLevel = DEFAULT_ESCORT_MAIN_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("mainLoopRate", mainLoopRate)) {
        if(logLevel <= Warn) {
            ROS_WARN("EscortMain: Value of mainLoopRate not found, using default: %f", DEFAULT_MAIN_LOOP_RATE);
        }
        mainLoopRate = DEFAULT_MAIN_LOOP_RATE;
    }
    mainLoopTime = 1/mainLoopRate;
    //Modules initialization
    if(DataStorage::GetInstance().Initialize(nodeHandlePrivate)) {
        if(logLevel <= Debug) {
            ROS_DEBUG("EscortMain: Data storage initialized successfully");
        }
    }
    else {
        if(logLevel <= Error) {
            ROS_ERROR("EscortMain: Failed to initialize data storage");
        }
        return false;
    }
    if(MobilityModule::GetInstance().Initialize(nodeHandlePublic, nodeHandlePrivate)) {
        if(logLevel <= Debug) {
            ROS_DEBUG("EscortMain: Mobility module initialized successfully");
        }
    }
    else {
        if(logLevel <= Error) {
            ROS_ERROR("EscortMain: Failed to initialize mobility module");
        }
        return false;
    }
    if(SensorsModule::GetInstance().Initialize(nodeHandlePrivate)) {
        if(logLevel <= Debug) {
            ROS_DEBUG("EscortMain: Sensors module initialized successfully");
        }
    }
    else {
        if(logLevel <= Error) {
            ROS_ERROR("EscortMain: Failed to initialize sensors module");
        }
        return false;
    }
    if(IdentificationModule::GetInstance().Initialize(nodeHandlePrivate)) {
        if(logLevel <= Debug) {
            ROS_DEBUG("EscortMain: Identification module initialized successfully");
        }
    }
    else {
        if(logLevel <= Error) {
            ROS_ERROR("EscortMain: Failed to initialize identification module");
        }
        return false;
    }
    //TODO: inicjalizacja modułu topica
    if(TaskModule::GetInstance().Initialize(nodeHandlePrivate)) {
        if(logLevel <= Debug) {
            ROS_DEBUG("EscortMain: Task module initialized successfully");
        }
    }
    else {
        if(logLevel <= Error) {
            ROS_ERROR("EscortMain: Failed to initialize task module");
        }
        return false;
    }
    if(logLevel <= Info) {
        ROS_INFO("EscortMain: Initialization complete, starting program");
    }
	return true;
}

void Update() {
    //TODO: topic module update
    SensorsModule::GetInstance().Update();
    IdentificationModule::GetInstance().Update();
    TaskModule::GetInstance().Update();
    MobilityModule::GetInstance().Update(mainLoopTime);
    DataStorage::GetInstance().Update(mainLoopTime);
}

void Finish() {
	delete nodeHandlePublic;
	delete nodeHandlePrivate;
    SensorsModule::GetInstance().Finish();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "escort_main");
	if(Initialization()) {
		ros::Rate mainLoopRate(mainLoopRate);
		while (ros::ok()) {
            Update();
			mainLoopRate.sleep();
		}
	}
    if(logLevel <= Info) {
        ROS_INFO("EscortMain: Ending program");
    }
	Finish();
	return 0;
}
