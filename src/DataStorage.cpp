#include "DataStorage.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataStorage::Initialize(ros::NodeHandle* nodeHandlePrivate)
{
    int _logLevel;
    if(!nodeHandlePrivate->getParam("dataStorageLogLevel", _logLevel))
    {
        ROS_WARN("dataStorageLogLevel not found, using default");
        logLevel = DEFAULT_DATA_STORAGE_LOG_LEVEL;
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
                ROS_WARN("Requested invalid dataStorageLogLevel, using default");
                logLevel = DEFAULT_DATA_STORAGE_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("maxUsers", maxUsers))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of maxUsers not found, using default: %d", DEFAULT_MAX_USERS);
        }
        maxUsers = DEFAULT_MAX_USERS;
    }
    if(maxUsers <= 0)
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Requested invalid number of max users: %d", maxUsers);
        }
        maxUsers = 1;
    }
    poseCooldown.resize(maxUsers);
    poseDetected.resize(maxUsers);
    centerOfMassLocation.resize(maxUsers);
    XnPoint3D zero;
    zero.X = 0.0f;
    zero.Y = 0.0f;
    zero.Z = 0.0f;
    for(int i=0; i<maxUsers; ++i)
    {
        poseCooldown.push_back(0.0f);
        poseDetected.push_back(false);
        centerOfMassLocation.push_back(zero);
    }
    if(!nodeHandlePrivate->getParam("poseCooldownTime", poseCooldownTime))
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Value of poseCooldownTime not found, using default: %d", DEFAULT_POSE_COOLDOWN_TIME);
        }
        poseCooldownTime = DEFAULT_POSE_COOLDOWN_TIME;
    }
    if(poseCooldownTime < 0.0f)
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Requested negative time of pose cooldown: %f", poseCooldownTime);
        }
        poseCooldownTime = 0.0f;
    }
    currentUserXnId = NO_USER;
    return true;
}

void DataStorage::Update(float timeElapsed)
{
    UpdatePoseData(timeElapsed);
    ClearCenterOfMasses();
}

int DataStorage::GetMaxUsers()
{
    return maxUsers;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
XnUserID DataStorage::GetCurrentUserXnId()
{
    return currentUserXnId;
}

void DataStorage::SetCurrentUserXnId(XnUserID newCurrentUserXnId)
{
    currentUserXnId = newCurrentUserXnId;
}

void DataStorage::PoseDetectedForUser(int userId)
{
    if(userId < 0 || userId >= poseDetected.size())
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Attempt to change pose detected for invalid user: %d", userId);
        }
        return;
    }
    else
    {
        if(poseCooldown[userId] > 0.0f)
        {
            if(logLevel <= Debug)
            {
                ROS_DEBUG("Pose detected for user %d, but ignored due to cooldown", userId);
            }
        }
        else
        {
            poseDetected[userId] = true;
            if(logLevel <= Debug)
            {
                ROS_DEBUG("Pose detected for user %d", userId);
            }
        }
    }
}

XnPoint3D DataStorage::GetCenterOfMassLocationForUser(int userId)
{
    if(userId < 0 || userId >= centerOfMassLocation.size())
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Attempt to get center of mass of invalid user: %d", userId);
        }
        XnPoint3D result;
        result.X = 0.0f;
        result.Y = 0.0f;
        result.Z = 0.0f;
        return result;
    }
    else
    {
        return centerOfMassLocation[userId];
    }
}

void DataStorage::SetCenterOfMassLocationForUser(int userId, XnPoint3D CoMLocation)
{
    if(userId < 0 || userId >= centerOfMassLocation.size())
    {
        if(logLevel <= Warn)
        {
            ROS_WARN("Attempt to get center of mass of invalid user: %d", userId);
        }
    }
    else
    {
        centerOfMassLocation[userId] = CoMLocation;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataStorage::UpdatePoseData(float timeElapsed)
{
    for(int i=0; i<maxUsers; ++i)
    {
        if(poseCooldown[i] > 0.0f)
        {
            poseCooldown[i] -= timeElapsed;
            if(poseCooldown[i] < 0.0f)
            {
                poseCooldown[i] = 0.0f;
            }
        }
        poseDetected[i] = false;
    }
}

void DataStorage::ClearCenterOfMasses()
{
    XnPoint3D zero;
    zero.X = 0.0f;
    zero.Y = 0.0f;
    zero.Z = 0.0f;
    for(int i=0; i<maxUsers; ++i)
    {
        centerOfMassLocation[i] = zero;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...