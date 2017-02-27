#ifndef ELEKTRON_ESCORT_MOBILITY_MODULE_H
#define ELEKTRON_ESCORT_MOBILITY_MODULE_H

#define DEFAULT_MOBILITY_MODULE_LOG_LEVEL Info
#define DEFAULT_DISTANCE_TO_KEEP 2500
#define DEFAULT_MAX_LINEAR_SPEED 0.254
#define DEFAULT_MAX_LINEAR_SPEED_DISTANCE 4000
#define DEFAULT_POSITION_TOLERANCE 100
#define DEFULT_MAX_FOLLOWING_TURNING_SPEED 0.36
#define DEFAULT_MAX_FOLLOWING_TURNING_SPEED_DISTANCE 2000
#define DEFULT_SEARCHING_TURNING_SPEED 0.12

#define DRIVES_TOPIC_NAME "cmd_vel_absolute"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <XnTypes.h>
#include "../Common.h"
#include "DataStorage.h"
#include "SensorsModule.h"


enum DrivesState {
    Stop, FollowUser, SearchForUser
};

class MobilityModule {
public:
    static MobilityModule &GetInstance() {
        static MobilityModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePublic, ros::NodeHandle *nodeHandlePrivate);
    void Update();
    void SetState(DrivesState newState);

private:
    LogLevels logLevel;
    DrivesState state;
    ros::Publisher publisher;
    double distanceToKeep;
    double maxLinearSpeed;
    double maxLinearSpeedDistance;
    double positionTolerance;
    double maxFollowingTurningSpeed;
    double maxFollowingTurningSpeedDistance;
    double searchingTurningSpeed;

    MobilityModule() {}
    MobilityModule(const MobilityModule &);
    MobilityModule &operator=(const MobilityModule &);
    ~MobilityModule() {}
    void StopStateUpdate();
    void FollowUserStateUpdate();
    void SearchForUserStateUpdate();
};

#endif //ELEKTRON_ESCORT_MOBILITY_MODULE_H
