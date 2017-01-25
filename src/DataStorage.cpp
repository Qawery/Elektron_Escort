#include "DataStorage.h"

bool DataStorage::Initialize(ros::NodeHandle* nodeHandlePrivate)
{
    int _maxUsers;
    if(!nodeHandlePrivate->getParam("maxUsers", _maxUsers))
    {
        ROS_WARN("Value of maxUsers not found, using default: %d.", DEFAULT_MAX_USERS);
        _maxUsers = DEFAULT_MAX_USERS;
    }
    SetMaxUsers(_maxUsers);
    double _poseCooldownTime;
    if(!nodeHandlePrivate->getParam("poseCooldownTime", _poseCooldownTime))
    {
        ROS_WARN("Value of poseCooldownTime not found, using default: %d.", DEFAULT_POSE_COOLDOWN_TIME);
        _poseCooldownTime = DEFAULT_POSE_COOLDOWN_TIME;
    }
    SetPoseCooldownTime(_poseCooldownTime);
    return true;
}

void DataStorage::UpdatePoseCooldowns(float timeElapsed)
{
    for(int i=0; i<maxUsers; ++i)
    {
        if(poseCooldownForUsers[i] > 0)
        {
            poseCooldownForUsers[i] -= timeElapsed;
            if(poseCooldownForUsers[i] < 0)
            {
                poseCooldownForUsers[i] = 0;
            }
        }
    }
}

int DataStorage::GetMaxUsers()
{
    return maxUsers;
}

void DataStorage::SetMaxUsers(int _maxUsers)
{
    if(poseCooldownForUsers.max_size() < _maxUsers)
    {
        maxUsers = poseCooldownForUsers.max_size();
        ROS_WARN("Requested too large number of max users: %d. Available number: %d", _maxUsers, maxUsers);
    }
    else if(_maxUsers <= 0)
    {
        ROS_ERROR("Requested invalid number of max users: %d", _maxUsers);
        //TODO: wywalenie programu
        maxUsers = 1;
    }
    else
    {
        maxUsers = _maxUsers;
    }
    poseCooldownForUsers.clear();
    poseCooldownForUsers.resize(maxUsers);
    for(int i=0; i<maxUsers; ++i)
    {
        poseCooldownForUsers.push_back(0.0f);
    }
}

void DataStorage::SetPoseCooldownTime(float _poseCooldownTime)
{
    if(_poseCooldownTime < 0)
    {
        ROS_WARN("Requested negative value of pose cooldown time: %f", _poseCooldownTime);
        poseCooldownTime = 0;
    }
    else
    {
        poseCooldownTime = _poseCooldownTime;
    }
    for(int i=0; i<maxUsers; ++i)
    {
        if(poseCooldownForUsers[i] > poseCooldownTime)
        {
            poseCooldownForUsers[i] = poseCooldownTime;
        }
    }
}

float DataStorage::GetPoseCooldownForUser(int _userNumber)
{
    if(_userNumber < 0 || _userNumber >= poseCooldownForUsers.size())
    {
        ROS_WARN("Requested pose cooldown for invalid user: %d", _userNumber);
        return 0;
    }
    return poseCooldownForUsers[_userNumber];
}