#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#include <ros/ros.h>
#include <XnOpenNI.h>
#include "DefaultValues.h"

class DataStorage
{
public:
    static DataStorage& GetInstance()
    {
        static DataStorage instance;
        return instance;
    }

    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void Update(float timeElapsed);
    int GetMaxUsers();
    void SetMaxUsers(int _maxUsers);
    void SetPoseCooldownTime(float _poseCooldownTime);
    float GetPoseCooldownForUser(int _userNumber);
    XnUserID  GetCurrentUserId();
    void SetCurrentUserId(XnUserID _currentUserId);
    bool GetPoseDetectedForUser(int userId);
    void PoseDetectedForUser(int userId);

private:
    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
    void UpdatePoseCooldowns(float timeElapsed);
    void UpdatePoseDetected();

    int maxUsers;
    float poseCooldownTime;
    std::vector<float> poseCooldownForUsers;
    std::vector<bool> poseDetectedForUsers;

    XnUserID currentUserId;
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
