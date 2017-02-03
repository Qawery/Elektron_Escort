#include "DataStorage.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
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
    userNew.resize(maxUsers);
    userExit.resize(maxUsers);
    userReEnter.resize(maxUsers);
    userLost.resize(maxUsers);
    userPose.resize(maxUsers);
    poseCooldown.resize(maxUsers);
    centerOfMassLocation.resize(maxUsers);
    XnPoint3D zero;
    zero.X = 0.0f;
    zero.Y = 0.0f;
    zero.Z = 0.0f;
    for(int i=0; i<maxUsers; ++i) {
        userNew.push_back(false);
        userExit.push_back(false);
        userReEnter.push_back(false);
        userLost.push_back(false);
        userPose.push_back(false);
        poseCooldown.push_back(0.0f);
        centerOfMassLocation.push_back(zero);
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
    UpdateUserData(timeElapsed);
    ClearCenterOfMasses();
}

int DataStorage::GetMaxUsers() {
    return maxUsers;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
XnUserID DataStorage::GetCurrentUserXnId() {
    return currentUserXnId;
}

void DataStorage::SetCurrentUserXnId(XnUserID newCurrentUserXnId) {
    currentUserXnId = newCurrentUserXnId;
}

void DataStorage::UserNew(int userId) {
    if(userId < 0 || userId >= userNew.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Invalid user new: %d", userId);
        }
        return;
    }
    else {
        if(logLevel <= Debug) {
            ROS_WARN("DataStorage: User new: %d", userId);
        }
        userNew[userId] = true;
    }
}

bool DataStorage::IsUserNew(int userId) {
    if(userId < 0 || userId >= userNew.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to read invalid user new: %d", userId);
        }
        return false;
    }
    else {
        return userNew[userId];
    }
}

void DataStorage::UserExit(int userId) {
    if(userId < 0 || userId >= userExit.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Invalid user exit: %d", userId);
        }
        return;
    }
    else {
        if(logLevel <= Debug) {
            ROS_WARN("DataStorage: User exit: %d", userId);
        }
        userExit[userId] = true;
    }
}

bool DataStorage::IsUserExit(int userId) {
    if(userId < 0 || userId >= userExit.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to read invalid user exit: %d", userId);
        }
        return false;
    }
    else {
        return userExit[userId];
    }
}

void DataStorage::UserReEnter(int userId) {
    if(userId < 0 || userId >= userReEnter.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Invalid user re enter: %d", userId);
        }
        return;
    }
    else {
        if(logLevel <= Debug) {
            ROS_WARN("DataStorage: User re enter: %d", userId);
        }
        userReEnter[userId] = true;
    }
}

bool DataStorage::IsUserReEnter(int userId) {
    if(userId < 0 || userId >= userReEnter.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to read invalid user reenter: %d", userId);
        }
        return false;
    }
    else {
        return userReEnter[userId];
    }
}

void DataStorage::UserLost(int userId) {
    if(userId < 0 || userId >= userLost.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Invalid user Lost: %d", userId);
        }
        return;
    }
    else {
        if(logLevel <= Debug) {
            ROS_WARN("DataStorage: User lost: %d", userId);
        }
        userLost[userId] = true;
    }
}

bool DataStorage::IsUserLost(int userId) {
    if(userId < 0 || userId >= userLost.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to read invalid user lost: %d", userId);
        }
        return false;
    }
    else {
        return userLost[userId];
    }
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

XnPoint3D DataStorage::GetCenterOfMassLocationForUser(int userId) {
    if(userId < 0 || userId >= centerOfMassLocation.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to get center of mass of invalid user: %d", userId);
        }
        XnPoint3D result;
        result.X = 0.0f;
        result.Y = 0.0f;
        result.Z = 0.0f;
        return result;
    }
    else {
        return centerOfMassLocation[userId];
    }
}

void DataStorage::SetCenterOfMassLocationForUser(int userId, XnPoint3D CoMLocation) {
    if(userId < 0 || userId >= centerOfMassLocation.size()) {
        if(logLevel <= Warn) {
            ROS_WARN("DataStorage: Attempt to set center of mass of invalid user: %d", userId);
        }
    }
    else {
        centerOfMassLocation[userId] = CoMLocation;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void DataStorage::UpdateUserData(double timeElapsed) {
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
        userNew[i] = false;
        userExit[i] = false;
        userReEnter[i] = false;
        userLost[i] = false;
        userPose[i] = false;
    }
}

void DataStorage::ClearCenterOfMasses() {
    XnPoint3D zero;
    zero.X = 0.0f;
    zero.Y = 0.0f;
    zero.Z = 0.0f;
    for(int i=0; i<maxUsers; ++i) {
        centerOfMassLocation[i] = zero;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...