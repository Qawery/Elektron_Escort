#include "IdentificationModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool IdentificationModule::Initialize(ros::NodeHandle *nodeHandlePrivate) {
    int _logLevel;
    if(!nodeHandlePrivate->getParam("identificationModuleLogLevel", _logLevel)) {
        ROS_WARN("IdentificationModule: Log level not found, using default");
        logLevel = DEFAULT_IDENTIFICATION_MODULE_LOG_LEVEL;
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
                ROS_WARN("IdentificationModule: Requested invalid log level, using default");
                logLevel = DEFAULT_IDENTIFICATION_MODULE_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("identificationThreshold", identificationThreshold)) {
        if(logLevel <= Warn) {
            ROS_WARN("IdentificationModule: Value of identificationThreshold not found, using default: %f", DEFAULT_IDENTIFICATION_THRESHOLD);
        }
        identificationThreshold = DEFAULT_IDENTIFICATION_THRESHOLD;
    }
    if(!nodeHandlePrivate->getParam("userID_MethodTrust", userID_method.trustValue)) {
        if(logLevel <= Warn) {
            ROS_WARN("IdentificationModule: Trust value for userID method not found, using default: %f", DEFAULT_USER_ID_METHOD_TRUST);
        }
        userID_method.trustValue = DEFAULT_USER_ID_METHOD_TRUST;
    }
    if(logLevel <= Info) {
        ROS_INFO("IdentificationModule: initialized");
    }
    isTemplateSaved = false;
    return true;
}

void IdentificationModule::Update()
{
    if(isTemplateSaved) {
        //TODO: pozostałe metody
        userID_method.Update();
        XnUInt16 numberOfUsers = SensorsModule::GetInstance().GetUserGenerator().GetNumberOfUsers();
        XnUserID userIds[numberOfUsers];
        SensorsModule::GetInstance().GetUserGenerator().GetUsers(userIds, numberOfUsers);
        float usersRanking[numberOfUsers];
        for (int i = 0; i < numberOfUsers; ++i) {
            usersRanking[i] = 0.0f;
            if(DataStorage::GetInstance().IsPresentOnScene(userIds[i])) {
                //TODO: pozostałe metody
                usersRanking[i] += userID_method.trustValue * userID_method.RateUser(userIds[i]);
            }
        }
        int favouriteIndex = 0;
        for (int i = 1; i < numberOfUsers; ++i) {
            if (usersRanking[favouriteIndex] < usersRanking[i]) {
                favouriteIndex = i;
            }
        }
        if (usersRanking[favouriteIndex] > identificationThreshold) {
            DataStorage::GetInstance().SetCurrentUserXnId(userIds[favouriteIndex]);
        } else {
            DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
        }
        //TODO: pozostałe metody
        userID_method.LateUpdate();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void IdentificationModule::ClearTemplate() {
    userID_method.ClearTemplate();
    isTemplateSaved = false;
}

bool IdentificationModule::SaveTemplateOfCurrentUser() {
    if(userID_method.SaveTemplate()) {
        isTemplateSaved = true;
        return true;
    }
    else {
        return false;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...