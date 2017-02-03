#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#define DEFAULT_DATA_STORAGE_LOG_LEVEL Info
#define DEFAULT_MAX_USERS 3
#define DEFAULT_POSE_COOLDOWN_TIME 3.0f

#include <mutex>
#include <ros/ros.h>
#include <XnCppWrapper.h>
#include "../Common.h"


class DataStorage {
public:
    //System functions
    static DataStorage& GetInstance() {
        static DataStorage instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void Update(double timeElapsed);
    //TODO: wiadomości sterujące
    int GetMaxUsers();

    //Task functions
    XnUserID GetCurrentUserXnId();
    void SetCurrentUserXnId(XnUserID newCurrentUserXnId);
    void UserNew(int userId);
    bool IsUserNew(int userId);
    void UserExit(int userId);
    bool IsUserExit(int userId);
    void UserReEnter(int userId);
    bool IsUserReEnter(int userId);
    void UserLost(int userId);
    bool IsUserLost(int userId);
    void UserPose(int userId);
    bool IsUserPose(int userId);
    bool IsPoseCooldownPassed(int userId);
    void SetCenterOfMassLocationForUser(int userId, XnPoint3D CoMLocation);
    XnPoint3D GetCenterOfMassLocationForUser(int userId);

private:
    //System fields
    LogLevels logLevel;
    int maxUsers;
    float poseCooldownTime;
    //TODO: czy jest nowa wiadomość sterująca
    //TODO: wiadomość sterująca

    //Task fields
    XnUserID currentUserXnId;
    std::vector<bool> userNew;
    std::vector<bool> userExit;
    std::vector<bool> userReEnter;
    std::vector<bool> userLost;
    std::vector<bool> userPose;
    std::vector<float> poseCooldown;
    std::vector<XnPoint3D> centerOfMassLocation;

    //System functions
    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
    void UpdateUserData(double timeElapsed);
    void ClearCenterOfMasses();

    //Task functions
    //...
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
