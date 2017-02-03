#ifndef ELEKTRON_ESCORT_MOBILITY_MODULE_H
#define ELEKTRON_ESCORT_MOBILITY_MODULE_H

#define DEFAULT_MOBILITY_MODULE_LOG_LEVEL Info
#define DEFAULT_MAX_LINEAR_SPEED 100
#define DEFAULT_MAX_LINEAR_SPEED_DISTANCE 1000
#define DEFULT_MAX_ANGULAR_SPEED 100
#define DEFAULT_MAX_ANGULAR_SPEED_DISTANCE 500
#define DEFAULT_POSITION_TOLERANCE 100
#define DEFAULT_DISTANCE_TO_KEEP 2000
#define DRIVES_TOPIC_NAME "cmd_vel"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include "../Common.h"
#include "DataStorage.h"


enum DrivesState
{
    Stop, FollowUser, SearchForUser
};

class MobilityModule {
public:
    //System functions
    static MobilityModule &GetInstance() {
        static MobilityModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePublic, ros::NodeHandle *nodeHandlePrivate);
    void Update();

    //Task functions
    DrivesState GetState();
    void SetState(DrivesState newState);

private:
    //System fields
    LogLevels logLevel;
    DrivesState state;
    ros::Publisher publisher;
    double distanceToKeep;
    double positionTolerance;
    double maxAngularSpeed;
    double maxLinearSpeed;
    double maxLinearSpeedDistance;
    double maxAngularSpeedDistance;

    //Task fields
    XnPoint3D lastUserLocation;

    //System functions
    MobilityModule() {}
    MobilityModule(const MobilityModule &);
    MobilityModule &operator=(const MobilityModule &);
    ~MobilityModule() {}

    //Task functions
    void StopStateUpdate();
    void FollowUserStateUpdate();
    void SearchForUserStateUpdate();
};

#endif //ELEKTRON_ESCORT_MOBILITY_MODULE_H
