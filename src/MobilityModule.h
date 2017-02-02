#ifndef ELEKTRON_ESCORT_MOBILITYMODULE_H
#define ELEKTRON_ESCORT_MOBILITYMODULE_H

#define DEFAULT_MOBILITY_MODULE_LOG_LEVEL Debug
#define DEFAULT_MAX_LINEAR_SPEED 100
#define DEFAULT_MAX_LINEAR_SPEED_DISTANCE 50
#define DEFULT_MAX_ANGULAR_SPEED 100
#define DEFAULT_MAX_ANGULAR_SPEED_DISTANCE 50
#define DEFAULT_POSITION_TOLERANCE 5
#define DEFAULT_DISTANCE_TO_KEEP 10
#define DRIVES_TOPIC_NAME "cmd_vel"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include "Common.h"
#include "DataStorage.h"


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
    void SetState(DrivesState newState);

private:
    //System fields
    LogLevels logLevel;

    //Task fields
    DrivesState state;
    geometry_msgs::Twist velocity;
    ros::Publisher publisher;
    double distanceToKeep;
    double positionTolerance;
    double maxAngularSpeed;
    double maxLinearSpeed;
    double maxLinearSpeedDistance;
    double maxAngularSpeedDistance;

    //System functions
    MobilityModule() {}
    MobilityModule(const MobilityModule &);
    MobilityModule &operator=(const MobilityModule &);
    ~MobilityModule() {}

    //Task functions
};

#endif //ELEKTRON_ESCORT_MOBILITYMODULE_H
