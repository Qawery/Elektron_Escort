#include "DataStorage.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DataStorage::Initialize(ros::NodeHandle* nodeHandlePrivate) {
    int _logLevel;
    if(!nodeHandlePrivate->getParam("dataStorageLogLevel", _logLevel)) {
        ROS_WARN("DataStorage: Log level not found, using default");
        logLevel = DEFAULT_DATA_STORAGE_LOG_LEVEL;
    }
    else {
        switch (_logLevel) {
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
                ROS_WARN("DataStorage: Requested invalid log level, using default");
                logLevel = DEFAULT_DATA_STORAGE_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("maxUsers", maxUsers)) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Value of maxUsers not found, using default: %d", DEFAULT_MAX_USERS);
        }
        maxUsers = DEFAULT_MAX_USERS;
    }
    if(maxUsers <= 0) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Requested invalid number of max users: %d", maxUsers);
        }
        maxUsers = 1;
    }
    userPose.resize(maxUsers);
    poseCooldown.resize(maxUsers);
    XnPoint3D zero;
    zero.X = 0.0f;
    zero.Y = 0.0f;
    zero.Z = 0.0f;
    lastUserPosition = zero;
    for(int i=0; i<maxUsers; ++i) {
        userPose.push_back(false);
        poseCooldown.push_back(0.0f);
    }
    if(!nodeHandlePrivate->getParam("poseCooldownTime", poseCooldownTime)) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Value of poseCooldownTime not found, using default: %f", DEFAULT_POSE_COOLDOWN_TIME);
        }
        poseCooldownTime = DEFAULT_POSE_COOLDOWN_TIME;
    }
    if(poseCooldownTime < 0.0f) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Requested negative time of pose cooldown: %f", poseCooldownTime);
        }
        poseCooldownTime = 0.0f;
    }
    currentUserXnId = NO_USER;
    if(logLevel <= Info) {
        ROS_INFO("DataStorage: initialized");
    }
    return true;
}

void DataStorage::Update(double timeElapsed) {
    for(int i=0; i<maxUsers; ++i) {
        if(poseCooldown[i] > 0.0f) {
            poseCooldown[i] -= timeElapsed;
            if(poseCooldown[i] < 0.0f) {
                poseCooldown[i] = 0.0f;
            }
        }
        if(logLevel <= Debug) {
            ROS_DEBUG("DataStorage: Pose cooldown for %d: %f", i, poseCooldown[i]);
        }
        userPose[i] = false;
    }
    if(currentUserXnId != NO_USER) {
        SensorsModule::GetInstance().GetUserGenerator().GetCoM(currentUserXnId, lastUserPosition);
    }
    std::set<XnUserID> toRemove;
    std::set<XnUserID>::iterator iter;
    for(iter=presentUsers.begin(); iter!=presentUsers.end(); ++iter) {
        XnPoint3D userCoM;
        SensorsModule::GetInstance().GetUserGenerator().GetCoM(*iter, userCoM);
        if(userCoM.Z <= 1.0) {
            toRemove.insert(*iter);
            if(logLevel <= Warn) {
                ROS_WARN("Deleted invalid user: %d", *iter);
            }
        }
    }
    for(iter=toRemove.begin(); iter!=toRemove.end(); ++iter) {
        presentUsers.erase(*iter);
    }
}

XnUserID DataStorage::GetCurrentUserXnId() {
    return currentUserXnId;
}

void DataStorage::SetCurrentUserXnId(XnUserID newCurrentUserXnId) {
    currentUserXnId = newCurrentUserXnId;
}

void DataStorage::UserNew(XnUserID userId) {
    presentUsers.insert(userId);
}

void DataStorage::UserExit(XnUserID userId) {
    presentUsers.erase(userId);
}

void DataStorage::UserReEnter(XnUserID userId) {
    presentUsers.insert(userId);
}

void DataStorage::UserPose(int userId) {
    if(userId < 0 || userId >= userPose.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to change pose detected for invalid user: %d", userId);
        }
        return;
    }
    else {
        if(poseCooldown[userId] > 0.0f) {
            if(logLevel <= Debug) {
                ROS_DEBUG("DataStorage: Pose detected for user %d, but ignored due to cooldown", userId);
            }
        }
        else {
            userPose[userId] = true;
            poseCooldown[userId] = poseCooldownTime;
            if(logLevel <= Debug) {
                ROS_DEBUG("DataStorage: Pose detected for user %d", userId);
            }
        }
    }
}

bool DataStorage::IsUserPose(int userId) {
    if(userId < 0 || userId >= userPose.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to read pose detected for invalid user: %d", userId);
        }
        return false;
    }
    else {
        return userPose[userId];
    }
}

bool DataStorage::IsPoseCooldownPassed(int userId) {
    if(userId < 0 || userId >= poseCooldown.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to read pose cooldown for invalid user: %d", userId);
        }
        return true;
    }
    else {
        if(poseCooldown[userId] <= 0.0f) {
            return true;
        }
        else {
            return false;
        }
    }
}

bool DataStorage::IsPresentOnScene(XnUserID userId) {
    if(presentUsers.find(userId) != presentUsers.end()) {
        return true;
    }
    else {
        return false;
    }
}

XnPoint3D DataStorage::GetLastUserPosition() {
    return lastUserPosition;
}

std::set<XnUserID>* DataStorage::GetPresentUsersSet() {
    return &presentUsers;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////