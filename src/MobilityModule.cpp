#include <XnTypes.h>
#include "MobilityModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MobilityModule::Initialize(ros::NodeHandle *nodeHandlePublic, ros::NodeHandle *nodeHandlePrivate)
{
    int _logLevel;
    if(!nodeHandlePrivate->getParam("mobilityModuleLogLevel", _logLevel))
    {
        ROS_WARN("mobilityModuleLogLevel not found, using default");
        logLevel = DEFAULT_MOBILITY_MODULE_LOG_LEVEL;
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
                ROS_WARN("Requested invalid mobilityModuleLogLevel, using default");
                logLevel = DEFAULT_MOBILITY_MODULE_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("distanceToKeep", distanceToKeep))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of distanceToKeep not found, using default: %d", DEFAULT_DISTANCE_TO_KEEP);
        }
        distanceToKeep = DEFAULT_DISTANCE_TO_KEEP;
    }
    if(!nodeHandlePrivate->getParam("positionTolerance", positionTolerance))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of positionTolerance not found, using default: %d", DEFAULT_POSITION_TOLERANCE);
        }
        positionTolerance = DEFAULT_POSITION_TOLERANCE;
    }
    if(!nodeHandlePrivate->getParam("maxAngularSpeed", maxAngularSpeed))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of maxAngularSpeed not found, using default: %d", DEFULT_MAX_ANGULAR_SPEED);
        }
        maxAngularSpeed = DEFULT_MAX_ANGULAR_SPEED;
    }
    if(!nodeHandlePrivate->getParam("maxLinearSpeed", maxLinearSpeed))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of maxLinearSpeed not found, using default: %d", DEFAULT_MAX_LINEAR_SPEED);
        }
        maxLinearSpeed = DEFAULT_MAX_LINEAR_SPEED;
    }
    if(!nodeHandlePrivate->getParam("maxLinearSpeedDistance", maxLinearSpeedDistance))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of maxLinearSpeedDistance not found, using default: %d", DEFAULT_MAX_LINEAR_SPEED_DISTANCE);
        }
        maxLinearSpeedDistance = DEFAULT_MAX_LINEAR_SPEED_DISTANCE;
    }
    if(!nodeHandlePrivate->getParam("maxAngularSpeedDistance", maxAngularSpeedDistance))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of maxAngularSpeedDistance not found, using default: %d", DEFAULT_MAX_ANGULAR_SPEED_DISTANCE);
        }
        maxAngularSpeedDistance = DEFAULT_MAX_ANGULAR_SPEED_DISTANCE;
    }
    publisher = nodeHandlePublic->advertise<geometry_msgs::Twist>(DRIVES_TOPIC_NAME, 1);
    return true;
}

void MobilityModule::Update()
{
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    if(state != Stop)
    {
        XnPoint3D currentUserLocation;
        if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER)
        {
            currentUserLocation = DataStorage::GetInstance().GetCenterOfMassLocationForUser(DataStorage::GetInstance().GetCurrentUserXnId()-1);
        }
        else
        {
            currentUserLocation.X = 0.0f;
            currentUserLocation.Y = 0.0f;
            currentUserLocation.Z = 0.0f;
            if(logLevel <= Warn)
            {
                ROS_WARN("No user, when expected");
            }
        }
        switch (state)
        {
            case FollowUser:
                if(currentUserLocation.Z > distanceToKeep)
                {
                    velocity.linear.x = ((currentUserLocation.Z-distanceToKeep)/maxLinearSpeedDistance)*maxLinearSpeed;
                    if(velocity.linear.x > maxLinearSpeed)
                    {
                        velocity.linear.x = maxLinearSpeed;
                    }
                }
                if(currentUserLocation.X > positionTolerance)
                {
                    velocity.angular.z = ((currentUserLocation.X-positionTolerance)/maxAngularSpeedDistance)*maxAngularSpeed;
                    if(velocity.angular.z > maxAngularSpeed)
                    {
                        velocity.angular.z = maxAngularSpeed;
                    }
                }
                else if(currentUserLocation.X < -positionTolerance)
                {
                    velocity.angular.z = ((currentUserLocation.X+positionTolerance)/maxAngularSpeedDistance)*maxAngularSpeed;
                    if(velocity.angular.z < -maxAngularSpeed)
                    {
                        velocity.angular.z = -maxAngularSpeed;
                    }
                }
                break;

            case SearchForUser:
                //TODO: poszukiwanie w kierunku, w którym zniknął użytkownik
                break;
        }
    }
    publisher.publish(velocity);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void MobilityModule::SetState(DrivesState newState)
{
    state = newState;
    Update();
    if(logLevel <= Debug)
    {
        ROS_DEBUG("New mobility state: %d", newState);
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
//...