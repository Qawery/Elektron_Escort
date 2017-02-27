#include "TaskModule.h"
#include "MobilityModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TaskModule::Initialize(ros::NodeHandle *nodeHandlePrivate) {
    int _logLevel;
    if(!nodeHandlePrivate->getParam("taskModuleLogLevel", _logLevel)) {
        ROS_WARN("TaskModule: Log level not found, using default");
        logLevel = DEFAULT_TASK_MODULE_LOG_LEVEL;
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
                ROS_WARN("TaskModule: Requested invalid log level, using default");
                logLevel = DEFAULT_TASK_MODULE_LOG_LEVEL;
                break;
        }
    }
    if(!nodeHandlePrivate->getParam("searchTimeLimit", searchTimeLimit)) {
        if(logLevel <= Warn) {
            ROS_WARN("TaskModule: Value of searchTimeLimit not found, using default: %f", DEFAULT_SEARCH_TIME_LIMIT);
        }
        searchTimeLimit = DEFAULT_SEARCH_TIME_LIMIT;
    }
    if(!nodeHandlePrivate->getParam("waitTimeLimit", waitTimeLimit)) {
        if(logLevel <= Warn) {
            ROS_WARN("TaskModule: Value of waitTimeLimit not found, using default: %f", DEFAULT_WAIT_TIME_LIMIT);
        }
        waitTimeLimit = DEFAULT_WAIT_TIME_LIMIT;
    }
    if(!nodeHandlePrivate->getParam("maxUserDistance", maxUserDistance)) {
        if(logLevel <= Warn) {
            ROS_WARN("TaskModule: Value of maxUserDistance not found, using default: %f", DEFAULT_MAX_USER_DISTANCE);
        }
        maxUserDistance = DEFAULT_MAX_USER_DISTANCE;
    }
    timeElapsed = 0.0;
    state = Idle;
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Initialized");
    }
    AwaitingStateEnter();
    return true;
}

void TaskModule::Update(double _timeElapsed) {
    switch (state) {
        case Idle:
            IdleStateUpdate();
            break;
        case Awaiting:
            AwaitingStateUpdate();
            break;
        case Saving:
            SavingTemplateUpdate();
            break;
        case Following:
            FollowingStateUpdate();
            break;
        case Waiting:
            timeElapsed += _timeElapsed;
            WaitingStateUpdate();
            break;
        case Searching:
            timeElapsed += _timeElapsed;
            SearchingStateUpdate();
            break;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//State change
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskModule::IdleStateEnter() {
    state = Idle;
    MobilityModule::GetInstance().SetState(Stop);
    IdentificationModule::GetInstance().ClearTemplate();
    SensorsModule::GetInstance().TurnSensorOff();
    DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Becoming idle");
    }
}

void TaskModule::AwaitingStateEnter() {
    switch (state) {
        case TaskState::Idle:
            SensorsModule::GetInstance().BeginCalibration();
            break;
        case TaskState::Saving:
            SensorsModule::GetInstance().ResetCalibration();
            DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
            break;
        case TaskState::Following:
            MobilityModule::GetInstance().SetState(Stop);
            IdentificationModule::GetInstance().ClearTemplate();
            DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
            SensorsModule::GetInstance().TurnSensorOff();
            SensorsModule::GetInstance().BeginCalibration();
            break;
        case TaskState::Searching:
            MobilityModule::GetInstance().SetState(Stop);
            IdentificationModule::GetInstance().ClearTemplate();
            DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
            SensorsModule::GetInstance().TurnSensorOff();
            SensorsModule::GetInstance().BeginCalibration();
            break;
    }
    state = Awaiting;
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Awaiting for user registration");
    }
}

void TaskModule::SavingStateEnter() {
    state = Saving;
    IdentificationModule::GetInstance().SaveTemplateOfCurrentUser();
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Saving template");
    }
}

void TaskModule::FollowingStateEnter() {
    switch (state) {
        case Saving:
            MobilityModule::GetInstance().SetState(FollowUser);
            SensorsModule::GetInstance().Work();
            break;
        case Waiting:
            MobilityModule::GetInstance().SetState(FollowUser);
            break;
        case Searching:
            MobilityModule::GetInstance().SetState(FollowUser);
            break;
    }
    state = Following;
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::WaitingStateEnter() {
    state = Waiting;
    timeElapsed = 0.0;
    MobilityModule::GetInstance().SetState(Stop);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Waiting for user");
    }
}

void TaskModule::SearchingStateEnter() {
    state = Searching;
    timeElapsed = 0.0;
    MobilityModule::GetInstance().SetState(SearchForUser);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Searching for user");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//State update
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskModule::IdleStateUpdate() {
    if(false) {
        //TODO: przejście w stan rejestracji na podstawie danych z topica
        AwaitingStateEnter();
    }
}

void TaskModule::AwaitingStateUpdate() {
    if(false){
        //TODO: przejście w stan Idle na podstawie danych z topica
        IdleStateEnter();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        SavingStateEnter();
    }
}

void TaskModule::SavingTemplateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        IdleStateEnter();
    }
    else if(IdentificationModule::GetInstance().GetState() == NoTemplate) {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Failed template creation for user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
        }
        AwaitingStateEnter();
    }
    else if(IdentificationModule::GetInstance().GetState() == PresentTemplate){
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Successful template creation for user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
        }
        FollowingStateEnter();
    }
}

void TaskModule::FollowingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        IdleStateEnter();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER && DataStorage::GetInstance().IsUserPose(DataStorage::GetInstance().GetCurrentUserXnId() - 1)) {
        AwaitingStateEnter();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        WaitingStateEnter();
    }
}

void TaskModule::WaitingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        IdleStateEnter();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        FollowingStateEnter();
    }
    else {
        if(DataStorage::GetInstance().GetLastUserPosition().Z > maxUserDistance) {
            if(timeElapsed >= waitTimeLimit) {
                SearchingStateEnter();
            }
        }
        else {
            if( timeElapsed >= waitTimeLimit/2) {
                SearchingStateEnter();
            }
        }
    }
}

void TaskModule::SearchingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        IdleStateEnter();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER && timeElapsed >= searchTimeLimit) {
        AwaitingStateEnter();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        FollowingStateEnter();
    }
}