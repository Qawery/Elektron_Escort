#include "TaskModule.h"
#include "MobilityModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TaskModule::Initialize(ros::NodeHandle *nodeHandlePrivate) {
    int _logLevel;
    if(!nodeHandlePrivate->getParam("taskModuleLogLevel", _logLevel)) {
        ROS_WARN("taskModuleLogLevel not found, using default");
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
                ROS_WARN("Requested invalid taskModuleLogLevel, using default");
                logLevel = DEFAULT_TASK_MODULE_LOG_LEVEL;
                break;
        }
    }
    state = Idle;
    if(logLevel <= Info) {
        ROS_INFO("Task module initialized");
    }
    //TODO: inne przestawienie w rejestrację ???
    AwaitRegistration();
    return true;
}

void TaskModule::Update() {
    switch (state) {
        case Idle:
            IdleStateUpdate();
            break;
        case Awaiting:
            AwaitingStateUpdate();
            break;
        case Following:
            FollowingStateUpdate();
            break;
        case Searching:
            SearchingStateUpdate();
            break;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//...


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskModule::BecomeIdle() {
    state = Idle;
    MobilityModule::GetInstance().SetState(Stop);
    IdentificationModule::GetInstance().ClearTemplate();
    DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
    SensorsModule::GetInstance().ChangeStateTo(Off);
    if(logLevel <= Info) {
        ROS_INFO("Becoming idle");
    }
}

void TaskModule::AwaitRegistration() {
    state = Awaiting;
    MobilityModule::GetInstance().SetState(Stop);
    IdentificationModule::GetInstance().ClearTemplate();
    DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
    SensorsModule::GetInstance().ChangeStateTo(Calibrating);
    if(logLevel <= Info) {
        ROS_INFO("Awaiting for user registration");
    }
}

void TaskModule::BeginFollowing() {
    state = Following;
    MobilityModule::GetInstance().SetState(FollowUser);
    IdentificationModule::GetInstance().SaveTemplateOfCurrentUser();
    if(logLevel <= Info) {
        ROS_INFO("Begin following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::ResumeFollowing() {
    state = Following;
    MobilityModule::GetInstance().SetState(FollowUser);
    if(logLevel <= Info) {
        ROS_INFO("Resuming following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::Search() {
    state = Searching;
    MobilityModule::GetInstance().SetState(SearchForUser);
    if(logLevel <= Info) {
        ROS_INFO("Searching for user");
    }
}

void TaskModule::IdleStateUpdate() {
    if(false) {
        //TODO: przejście w stan rejestracji na podstawie danych z topica
        AwaitRegistration();
    }
    else if(logLevel <= Debug) {
        ROS_DEBUG("state == %d", state);
        ROS_DEBUG("Idle. CurrentUserXnId= %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::AwaitingStateUpdate() {
    if(false){
        //TODO: przejście w stan Idle na podstawie danych z topica
        BecomeIdle();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        BeginFollowing();
    }
    else {
        if(logLevel <= Debug) {
            ROS_DEBUG("state == %d", state);
            ROS_DEBUG("Awaiting registration");
        }
    }
}

void TaskModule::FollowingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        BecomeIdle();
    }
    else if(false ||
            (DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER &&
             DataStorage::GetInstance().WasPoseDetectedForUser(DataStorage::GetInstance().GetCurrentUserXnId()-1))) {
        //TODO: przejście w stan rejestracji na podstawie danych z topica
        AwaitRegistration();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        Search();
    }
    else {
        if(logLevel <= Debug) {
            ROS_DEBUG("state == %d", state);
            ROS_DEBUG("Following. CurrentUserXnId= %d", DataStorage::GetInstance().GetCurrentUserXnId());
        }
    }
}

void TaskModule::SearchingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        BecomeIdle();
    }
    else if(false || (DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER && MobilityModule::GetInstance().GetState() == Stop)) {
        //TODO: przejście w stan rejestracji na podstawie danych z topica
        AwaitRegistration();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        ResumeFollowing();
    }
    else {
        if(logLevel <= Debug) {
            ROS_DEBUG("state == %d", state);
            ROS_DEBUG("Searching");
        }
    }
}