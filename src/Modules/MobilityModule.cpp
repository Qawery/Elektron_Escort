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
    if(!nodeHandlePrivate->getParam("positionTolerance", positionTolerance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of positionTolerance not found, using default: %d", DEFAULT_POSITION_TOLERANCE);
        }
        positionTolerance = DEFAULT_POSITION_TOLERANCE;
    }
    if(!nodeHandlePrivate->getParam("maxAngularSpeed", maxAngularSpeed)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxAngularSpeed not found, using default: %d", DEFULT_MAX_ANGULAR_SPEED);
        }
        maxAngularSpeed = DEFULT_MAX_ANGULAR_SPEED;
    }
    if(!nodeHandlePrivate->getParam("maxAngularSpeedDistance", maxAngularSpeedDistance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxAngularSpeedDistance not found, using default: %d", DEFAULT_MAX_ANGULAR_SPEED_DISTANCE);
        }
        maxAngularSpeedDistance = DEFAULT_MAX_ANGULAR_SPEED_DISTANCE;
    }
    if(!nodeHandlePrivate->getParam("maxLinearSpeed", maxLinearSpeed)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxLinearSpeed not found, using default: %d", DEFAULT_MAX_LINEAR_SPEED);
        }
        maxLinearSpeed = DEFAULT_MAX_LINEAR_SPEED;
    }
    if(!nodeHandlePrivate->getParam("maxLinearSpeedDistance", maxLinearSpeedDistance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxLinearSpeedDistance not found, using default: %d", DEFAULT_MAX_LINEAR_SPEED_DISTANCE);
        }
        maxLinearSpeedDistance = DEFAULT_MAX_LINEAR_SPEED_DISTANCE;
    }
    publisher = nodeHandlePublic->advertise<geometry_msgs::Twist>(DRIVES_TOPIC_NAME, 1);
    state = Stop;
    if(logLevel <= Info) {
        ROS_INFO("MobilityModule: initialized");
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
    if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        geometry_msgs::Twist velocity;
        velocity.linear.x = 0;
        velocity.angular.z = 0;
        publisher.publish(velocity);
        if(logLevel <= Warn){
            ROS_WARN("MobilityModule: No user to follow");
        }
    }
    else {
        geometry_msgs::Twist velocity;
        XnPoint3D currentUserLocation;
        currentUserLocation.X = 0.0f;
        currentUserLocation.Z = 0.0f;
        SensorsModule::GetInstance().GetUserGenerator().GetCoM(DataStorage::GetInstance().GetCurrentUserXnId(), currentUserLocation);
        if(currentUserLocation.Z > distanceToKeep) {
            velocity.linear.x = ((currentUserLocation.Z-distanceToKeep)/maxLinearSpeedDistance)*maxLinearSpeed;
            if(velocity.linear.x > maxLinearSpeed) {
                velocity.linear.x = maxLinearSpeed;
            }
        }
        if(currentUserLocation.X > positionTolerance) {
            velocity.angular.z = -(((currentUserLocation.X-positionTolerance)/maxAngularSpeedDistance)*maxAngularSpeed);
            if(velocity.angular.z < -maxAngularSpeed) {
                velocity.angular.z = -maxAngularSpeed;
            }
        }
        else if(currentUserLocation.X < -positionTolerance) {
            velocity.angular.z = -(((currentUserLocation.X+positionTolerance)/maxAngularSpeedDistance)*maxAngularSpeed);
            if(velocity.angular.z > maxAngularSpeed) {
                velocity.angular.z = maxAngularSpeed;
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
            velocity.angular.z = -maxAngularSpeed/2;
        }
        else {
            velocity.angular.z = maxAngularSpeed/2;
        }
    }
    publisher.publish(velocity);
}