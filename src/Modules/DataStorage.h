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

    //Task functions
    XnUserID GetCurrentUserXnId();
    void SetCurrentUserXnId(XnUserID newCurrentUserXnId);
    void UserNew(XnUserID userId);
    void UserExit(XnUserID userId);
    void UserReEnter(XnUserID userId);
    void UserPose(int userId);
    bool IsUserPose(int userId);
    bool IsPoseCooldownPassed(int userId);
    bool IsPresentOnScene(XnUserID userId);

private:
    //System fields
    LogLevels logLevel;
    int maxUsers;
    float poseCooldownTime;
    //TODO: czy jest nowa wiadomość sterująca
    //TODO: wiadomość sterująca

    //Task fields
    XnUserID currentUserXnId;
    std::vector<bool> userPose;
    std::vector<float> poseCooldown;
    std::set<XnUserID> presentUsers;

    //System functions
    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}
    void UpdateUserData(double timeElapsed);

    //Task functions
    //...
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
