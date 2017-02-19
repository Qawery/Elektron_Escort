#include "IdentificationModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
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
    methods[IM_UserId] = new UserID_Method();
    if(!nodeHandlePrivate->getParam("userID_MethodTrust", methods[IM_UserId]->trustValue)) {
        if(logLevel <= Warn) {
            ROS_WARN("IdentificationModule: Trust value for userID method not found, using default: %f", DEFAULT_USER_ID_METHOD_TRUST);
        }
        methods[IM_UserId]->trustValue = DEFAULT_USER_ID_METHOD_TRUST;
    }
    methods[IM_Height] = new Height_Method();
    if(!nodeHandlePrivate->getParam("height_MethodTrust", methods[IM_Height]->trustValue)) {
        if(logLevel <= Warn) {
            ROS_WARN("IdentificationModule: Trust value for height method not found, using default: %f", DEFAULT_USER_ID_METHOD_TRUST);
        }
        methods[IM_Height]->trustValue = DEFAULT_HEIGHT_METHOD_TRUST;
    }
    if(logLevel <= Info) {
        ROS_INFO("IdentificationModule: initialized");
    }
    state = NoTemplate;
    return true;
}

void IdentificationModule::Update()
{
    switch (state) {
        case IdentificationStates::NoTemplate:
            break;
        case IdentificationStates::SavingTemplate:
            ContinueSavingTemplate();
            break;
        case IdentificationStates::PresentTemplate:
            IdentifyUser();
            break;
    }
}

void IdentificationModule::Finish() {
    for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
        delete methods[i];
    }
}

void IdentificationModule::ClearTemplate() {
    for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
        methods[i]->ClearTemplate();
    }
    state = IdentificationStates::NoTemplate;
    if(logLevel <= Info) {
        ROS_INFO("IdentificationModule: template cleared");
    }
}

void IdentificationModule::SaveTemplateOfCurrentUser() {
    state = IdentificationStates::SavingTemplate;
    for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
        methods[i]->BeginSaveTemplate();
    }
    if(logLevel <= Info) {
        ROS_INFO("IdentificationModule: begin saving template");
    }
}

IdentificationStates IdentificationModule::GetState() {
    return state;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void IdentificationModule::ContinueSavingTemplate() {
    MethodState templateState = Ready;
    for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
        methods[i]->ContinueSaveTemplate();
        if(methods[i]->GetState() == CreatingTemplate) {
            templateState = CreatingTemplate;
        }
        else if (methods[i]->GetState() == NotReady) {
            templateState = NotReady;
            break;
        }
    }
    if(templateState == Ready) {
        state = IdentificationStates::PresentTemplate;
        if(logLevel <= Info) {
            ROS_INFO("IdentificationModule: saving template successful");
        }
    }
    else if (templateState == NotReady) {
        if(logLevel <= Info) {
            ROS_INFO("IdentificationModule: saving template failed");
        }
        ClearTemplate();
    }
}

void IdentificationModule::IdentifyUser() {
    for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
        methods[i]->Update();
    }
    /*
    //TODO: identyfikacja
    float usersRanking[numberOfUsers];
    if (numberOfUsers > 0) {
        for (int i = 0; i < numberOfUsers; ++i) {
            usersRanking[i] = 0.0f;
            if (DataStorage::GetInstance().IsPresentOnScene(userIds[i])) {
                for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
                    usersRanking[i] += methods[i]->trustValue * methods[i]->RateUser(userIds[i]);
                }
            }
        }
        int favouriteIndex = 0;
        for (int i = 1; i < numberOfUsers; ++i) {
            if (usersRanking[favouriteIndex] < usersRanking[i]) {
                favouriteIndex = i;
            }
        }
        if (usersRanking[favouriteIndex] >= identificationThreshold) {
            DataStorage::GetInstance().SetCurrentUserXnId(userIds[favouriteIndex]);
        }
        else {
            DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
        }
    }
    else {
        DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
    }
     */
    for( int i=0; i < IM_NUMBER_OF_METHODS; ++i) {
        methods[i]->LateUpdate();
    }
}