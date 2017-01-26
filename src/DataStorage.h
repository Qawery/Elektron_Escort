#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#include <mutex>
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
    void PoseDetectedForUser(int userId);

private:
    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
    void CopyNewData();
    void UpdatePoseCooldowns(float timeElapsed);
    void UpdatePoseDetected();

    int maxUsers;
    float poseCooldownTime;
    std::mutex newDataMutex;
    std::vector<float> poseCooldown;
    std::vector<bool> currentPoseDetected;
    std::vector<bool> newPoseDetected;
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
