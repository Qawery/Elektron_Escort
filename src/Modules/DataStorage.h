#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#define DEFAULT_DATA_STORAGE_LOG_LEVEL Info
#define DEFAULT_MAX_USERS 20
#define DEFAULT_POSE_COOLDOWN_TIME 3.0

#include <mutex>
#include <ros/ros.h>
#include <XnCppWrapper.h>
#include "../Common.h"
#include "SensorsModule.h"


class DataStorage {
public:
    static DataStorage& GetInstance() {
        static DataStorage instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void Update(double timeElapsed);
    XnUserID GetCurrentUserXnId();
    void SetCurrentUserXnId(XnUserID newCurrentUserXnId);
    void UserNew(XnUserID userId);
    void UserExit(XnUserID userId);
    void UserReEnter(XnUserID userId);
    void UserPose(int userId);
    bool IsUserPose(int userId);
    bool IsPoseCooldownPassed(int userId);
    bool IsPresentOnScene(XnUserID userId);
    XnPoint3D GetLastUserPosition();
    std::set<XnUserID>* GetPresentUsersSet();
    int GetMaxUsers();

private:
    LogLevels logLevel;
    int maxUsers;
    double poseCooldownTime;
    XnUserID currentUserXnId;
    std::vector<bool> userPose;
    std::vector<double> poseCooldown;
    std::set<XnUserID> presentUsers;
    XnPoint3D lastUserPosition;

    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
