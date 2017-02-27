#include "MobilityModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MobilityModule::Initialize(ros::NodeHandle *nodeHandlePublic, ros::NodeHandle *nodeHandlePrivate) {
    int _logLevel;
    if(!nodeHandlePrivate->getParam("mobilityModuleLogLevel", _logLevel)) {
        ROS_WARN("MobilityModule: Log level not found, using default");
        logLevel = DEFAULT_MOBILITY_MODULE_LOG_LEVEL;
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
                ROS_WARN("MobilityModule: Requested invalid log level, using default");
                logLevel = DEFAULT_MOBILITY_MODULE_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("distanceToKeep", distanceToKeep)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of distanceToKeep not found, using default: %d", DEFAULT_DISTANCE_TO_KEEP);
        }
        distanceToKeep = DEFAULT_DISTANCE_TO_KEEP;
    }
    if(!nodeHandlePrivate->getParam("maxLinearSpeed", maxLinearSpeed)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxLinearSpeed not found, using default: %f", DEFAULT_MAX_LINEAR_SPEED);
        }
        maxLinearSpeed = DEFAULT_MAX_LINEAR_SPEED;
    }
    if(!nodeHandlePrivate->getParam("maxLinearSpeedDistance", maxLinearSpeedDistance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxLinearSpeedDistance not found, using default: %d", DEFAULT_MAX_LINEAR_SPEED_DISTANCE);
        }
        maxLinearSpeedDistance = DEFAULT_MAX_LINEAR_SPEED_DISTANCE;
    }
    if(!nodeHandlePrivate->getParam("positionTolerance", positionTolerance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of positionTolerance not found, using default: %d", DEFAULT_POSITION_TOLERANCE);
        }
        positionTolerance = DEFAULT_POSITION_TOLERANCE;
    }
    if(!nodeHandlePrivate->getParam("maxFollowingTurningSpeed", maxFollowingTurningSpeed)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxFollowingTurningSpeed not found, using default: %f", DEFULT_MAX_FOLLOWING_TURNING_SPEED);
        }
        maxFollowingTurningSpeed = DEFULT_MAX_FOLLOWING_TURNING_SPEED;
    }
    if(!nodeHandlePrivate->getParam("maxFollowingTurningSpeedDistance", maxFollowingTurningSpeedDistance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxFollowingTurningSpeedDistance not found, using default: %d", DEFAULT_MAX_FOLLOWING_TURNING_SPEED_DISTANCE);
        }
        maxFollowingTurningSpeedDistance = DEFAULT_MAX_FOLLOWING_TURNING_SPEED_DISTANCE;
    }
    if(!nodeHandlePrivate->getParam("searchingTurningSpeed", searchingTurningSpeed)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of searchingTurningSpeed not found, using default: %f", DEFULT_SEARCHING_TURNING_SPEED);
        }
        searchingTurningSpeed = DEFULT_SEARCHING_TURNING_SPEED;
    }
    publisher = nodeHandlePublic->advertise<geometry_msgs::Twist>(DRIVES_TOPIC_NAME, 1);
    state = Stop;
    if(logLevel <= Info) {
        ROS_INFO("MobilityModule: Initialized");
    }
    return true;
}

void MobilityModule::Update() {
    switch (state) {
        case Stop:
            StopStateUpdate();
            break;
        case FollowUser:
            FollowUserStateUpdate();
            break;
        case SearchForUser:
            SearchForUserStateUpdate();
            break;
    }
}

void MobilityModule::SetState(DrivesState newState) {
    state = newState;
    if(logLevel <= Debug) {
        ROS_DEBUG("MobilityModule: New state: %d", newState);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void MobilityModule::StopStateUpdate() {
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    publisher.publish(velocity);
}

void MobilityModule::FollowUserStateUpdate() {
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        publisher.publish(velocity);
        if(logLevel <= Warn){
            ROS_WARN("MobilityModule: No user to follow");
        }
    }
    else {
        XnPoint3D currentUserLocation;
        SensorsModule::GetInstance().GetUserGenerator().GetCoM(DataStorage::GetInstance().GetCurrentUserXnId(), currentUserLocation);
        if(currentUserLocation.Z > distanceToKeep) {
            if(currentUserLocation.Z >= maxLinearSpeedDistance) {
                velocity.linear.x = maxLinearSpeed;
            }
            else {
                velocity.linear.x = (currentUserLocation.Z-distanceToKeep)/(maxLinearSpeedDistance-distanceToKeep);
                velocity.linear.x *= maxLinearSpeed;
            }
        }
        if(currentUserLocation.X > positionTolerance) {
            if(currentUserLocation.X >= maxFollowingTurningSpeedDistance) {
                velocity.angular.z = -maxFollowingTurningSpeed;
            }
            else {
                velocity.angular.z = (currentUserLocation.X-positionTolerance)/(maxFollowingTurningSpeedDistance-positionTolerance);
                velocity.angular.z *= -maxFollowingTurningSpeed;
            }
        }
        else if(currentUserLocation.X < -positionTolerance) {
            if(currentUserLocation.X <= -maxFollowingTurningSpeedDistance) {
                velocity.angular.z = maxFollowingTurningSpeed;
            }
            else {
                velocity.angular.z = (currentUserLocation.X+positionTolerance)/(maxFollowingTurningSpeedDistance-positionTolerance);
                velocity.angular.z *= -maxFollowingTurningSpeed;
            }
        }
        publisher.publish(velocity);
    }
}

void MobilityModule::SearchForUserStateUpdate() {
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER){
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: User present while searching");
        }
    }
    else {
        if(DataStorage::GetInstance().GetLastUserPosition().X >= 0) {
            velocity.angular.z = -searchingTurningSpeed;
        }
        else {
            velocity.angular.z = searchingTurningSpeed;
        }
    }
    publisher.publish(velocity);
}