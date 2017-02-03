#include "MobilityModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
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
    if(!nodeHandlePrivate->getParam("maxAngularSpeedDistance", maxAngularSpeedDistance)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of maxAngularSpeedDistance not found, using default: %d", DEFAULT_MAX_ANGULAR_SPEED_DISTANCE);
        }
        maxAngularSpeedDistance = DEFAULT_MAX_ANGULAR_SPEED_DISTANCE;
    }
    if(!nodeHandlePrivate->getParam("searchTimeLimit", searchTimeLimit)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of searchTimeLimit not found, using default: %f", DEFAULT_SEARCH_TIME_LIMIT);
        }
        searchTimeLimit = DEFAULT_SEARCH_TIME_LIMIT;
    }
    if(!nodeHandlePrivate->getParam("waitTime", waitTime)) {
        if(logLevel <= Warn) {
            ROS_WARN("MobilityModule: Value of waitTime not found, using default: %f", DEFAULT_WAIT_TIME);
        }
        waitTime = DEFAULT_WAIT_TIME;
    }
    searchTimeElapsed = 0.0f;
    publisher = nodeHandlePublic->advertise<geometry_msgs::Twist>(DRIVES_TOPIC_NAME, 1);
    state = Stop;
    if(logLevel <= Info) {
        ROS_INFO("MobilityModule: initialized");
    }
    return true;
}

void MobilityModule::Update(double timeElapsed) {
    switch (state) {
        case Stop:
            StopStateUpdate();
            break;
        case FollowUser:
            FollowUserStateUpdate();
            break;
        case SearchForUser:
            SearchForUserStateUpdate(timeElapsed);
            break;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
DrivesState MobilityModule::GetState() {
    return state;
}

void MobilityModule::SetState(DrivesState newState) {
    state = newState;
    searchTimeElapsed = 0.0f;
    if(logLevel <= Info) {
        ROS_INFO("MobilityModule: New state: %d", newState);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void MobilityModule::StopStateUpdate() {
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    if(logLevel <= Debug) {
        ROS_DEBUG("MobilityModule: state == %d", state);
        ROS_DEBUG("MobilityModule: Publishing velocity: linear_X= %f; angular_Z=  %f;", velocity.linear.x, velocity.angular.z);
    }
    publisher.publish(velocity);
}

void MobilityModule::FollowUserStateUpdate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        if(logLevel <= Debug){
            ROS_DEBUG("MobilityModule: User lost");
        }
        SetState(SearchForUser);
    }
    else {
        geometry_msgs::Twist velocity;
        lastUserLocation = DataStorage::GetInstance().GetCenterOfMassLocationForUser(DataStorage::GetInstance().GetCurrentUserXnId()-1);
        if(lastUserLocation.Z > distanceToKeep) {
            velocity.linear.x = ((lastUserLocation.Z-distanceToKeep)/maxLinearSpeedDistance)*maxLinearSpeed;
            if(velocity.linear.x > maxLinearSpeed) {
                velocity.linear.x = maxLinearSpeed;
            }
        }
        if(lastUserLocation.X > positionTolerance) {
            velocity.angular.z = ((lastUserLocation.X-positionTolerance)/maxAngularSpeedDistance)*maxAngularSpeed;
            if(velocity.angular.z > maxAngularSpeed) {
                velocity.angular.z = maxAngularSpeed;
            }
        }
        else if(lastUserLocation.X < -positionTolerance) {
            velocity.angular.z = ((lastUserLocation.X+positionTolerance)/maxAngularSpeedDistance)*maxAngularSpeed;
            if(velocity.angular.z < -maxAngularSpeed) {
                velocity.angular.z = -maxAngularSpeed;
            }
        }
        if(logLevel <= Debug) {
            ROS_DEBUG("MobilityModule: state == %d", state);
            ROS_DEBUG("MobilityModule: Publishing velocity: linear_X= %f; angular_Z=  %f;", velocity.linear.x, velocity.angular.z);
            ROS_DEBUG("MobilityModule: currentUserLocation: X= %f; Z= %f;", lastUserLocation.X, lastUserLocation.Z);
        }
        publisher.publish(velocity);
    }
}

void MobilityModule::SearchForUserStateUpdate(double timeElapsed) {
    geometry_msgs::Twist velocity;
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    searchTimeElapsed += timeElapsed;
    if(searchTimeElapsed >= searchTimeLimit && DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        if(logLevel <= Debug) {
            ROS_DEBUG("MobilityModule: Search failed");
        }
        SetState(Stop);
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER){
        if(logLevel <= Debug) {
            ROS_DEBUG("MobilityModule: User found");
        }
        SetState(FollowUser);
    }
    else {
        if(searchTimeElapsed > waitTime) {
            if(lastUserLocation.X > 0) {
                velocity.angular.z = maxAngularSpeed/2;
            }
            else
            {
                velocity.angular.z = -maxAngularSpeed/2;
            }
        }
        if(logLevel <= Debug) {
            ROS_DEBUG("MobilityModule: state == %d", state);
            ROS_DEBUG("MobilityModule: Publishing velocity: linear_X= %f; angular_Z=  %f;", velocity.linear.x, velocity.angular.z);
            ROS_DEBUG("MobilityModule: lastUserLocation: X= %f; Z= %f;", lastUserLocation.X, lastUserLocation.Z);
            ROS_DEBUG("MobilityModule: Search time: ");
        }
    }
    publisher.publish(velocity);
}