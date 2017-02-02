#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#define DEFAULT_DATA_STORAGE_LOG_LEVEL Debug
#define DEFAULT_MAX_USERS 3
#define DEFAULT_POSE_COOLDOWN_TIME 3

#include <mutex>
#include <ros/ros.h>
#include <XnCppWrapper.h>
#include "Common.h"


class DataStorage {
public:
    //System functions
    static DataStorage& GetInstance() {
        static DataStorage instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void Update(float timeElapsed);
    int GetMaxUsers();

    //Task functions
    XnUserID GetCurrentUserXnId();
    void SetCurrentUserXnId(XnUserID newCurrentUserXnId);
    void PoseDetectedForUser(int userId);
    void SetCenterOfMassLocationForUser(int userId, XnPoint3D CoMLocation);
    XnPoint3D GetCenterOfMassLocationForUser(int userId);

private:
    //System fields
    LogLevels logLevel;
    int maxUsers;
    float poseCooldownTime;

    //Task fields
    XnUserID currentUserXnId;
    std::vector<bool> poseDetected;
    std::vector<float> poseCooldown;
    std::vector<XnPoint3D> centerOfMassLocation;

    //System functions
    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
    void UpdatePoseData(float timeElapsed);
    void ClearCenterOfMasses();

    //Task functions
    //...
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
