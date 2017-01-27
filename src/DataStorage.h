#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#define DEFAULT_DATA_STORAGE_LOG_LEVEL Debug
#define DEFAULT_MAX_USERS 3
#define DEFAULT_POSE_COOLDOWN_TIME 3

#include <mutex>
#include <ros/ros.h>
#include <XnOpenNI.h>
#include "Common.h"

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
    void PoseDetectedForUser(XnUserID userId);
    int GetMaxUsers();

private:
    LogLevels logLevel;
    int maxUsers;
    float poseCooldownTime;
    std::vector<float> poseCooldown;
    std::vector<bool> poseDetected;

    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
    void UpdatePoseCooldowns(float timeElapsed);
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
