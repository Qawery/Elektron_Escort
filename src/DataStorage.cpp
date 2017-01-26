#include "DataStorage.h"

bool DataStorage::Initialize(ros::NodeHandle* nodeHandlePrivate)
{
    if(!nodeHandlePrivate->getParam("maxUsers", maxUsers))
    {
        ROS_WARN("Value of maxUsers not found, using default: %d.", DEFAULT_MAX_USERS);
        maxUsers = DEFAULT_MAX_USERS;
    }
    if(maxUsers <= 0)
    {
        ROS_WARN("Requested invalid number of max users: %d", maxUsers);
        maxUsers = 1;
    }
    poseCooldown.clear();
    poseCooldown.resize(maxUsers);
    currentPoseDetected.clear();
    currentPoseDetected.resize(maxUsers);
    newPoseDetected.clear();
    newPoseDetected.resize(maxUsers);
    for(int i=0; i<maxUsers; ++i)
    {
        poseCooldown.push_back(0.0f);
        currentPoseDetected.push_back(false);
        newPoseDetected.push_back(false);
    }
    if(!nodeHandlePrivate->getParam("poseCooldownTime", poseCooldownTime))
    {
        ROS_WARN("Value of poseCooldownTime not found, using default: %d.", DEFAULT_POSE_COOLDOWN_TIME);
        poseCooldownTime = DEFAULT_POSE_COOLDOWN_TIME;
    }
    if(poseCooldownTime < 0)
    {
        ROS_WARN("Requested negative time of pose cooldown: %d", poseCooldownTime);
        poseCooldownTime = 0.0f;
    }
    return true;
}

void DataStorage::Update(float timeElapsed)
{
    CopyNewData();
    UpdatePoseCooldowns(timeElapsed);
}

void DataStorage::CopyNewData()
{
    for(int i=0; i<currentPoseDetected.size(); ++i)
    {
        currentPoseDetected[i] = false;
    }
    newDataMutex.lock();
    for(int i=0; i<newPoseDetected.size(); ++i)
    {
        if(newPoseDetected[i])
        {
            if(poseCooldown[i] <= 0)
            {
                currentPoseDetected[i] = true;
                poseCooldown[i] = poseCooldownTime;
                newPoseDetected[i] = false;
            }
            else
            {
                ROS_DEBUG("Pose of user %d ignored due to cooldown.", i);
            }
        }
    }
    newDataMutex.unlock();
}

void DataStorage::UpdatePoseCooldowns(float timeElapsed)
{
    for(int i=0; i<maxUsers; ++i)
    {
        if(poseCooldown[i] > 0)
        {
            poseCooldown[i] -= timeElapsed;
            if(poseCooldown[i] < 0)
            {
                poseCooldown[i] = 0;
            }
        }
    }
}

void DataStorage::PoseDetectedForUser(int userId)
{
    newDataMutex.lock();
    if(userId < 0 || userId >= newPoseDetected.size())
    {
        ROS_WARN("Attempt to change pose detected for invalid user: %d", userId);
        return;
    }
    else
    {
        newPoseDetected[userId] = true;
    }
    newDataMutex.unlock();
}