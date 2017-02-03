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
    state = Idle;
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: initialized");
    }
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
        ROS_INFO("TaskModule: Becoming idle");
    }
}

void TaskModule::AwaitRegistration() {
    state = Awaiting;
    MobilityModule::GetInstance().SetState(Stop);
    IdentificationModule::GetInstance().ClearTemplate();
    DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
    SensorsModule::GetInstance().ChangeStateTo(Calibrating);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Awaiting for user registration");
    }
}

void TaskModule::BeginFollowing() {
    state = Following;
    MobilityModule::GetInstance().SetState(FollowUser);
    IdentificationModule::GetInstance().SaveTemplateOfCurrentUser();
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Begin following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::ResumeFollowing() {
    state = Following;
    MobilityModule::GetInstance().SetState(FollowUser);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Resuming following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::Search() {
    state = Searching;
    MobilityModule::GetInstance().SetState(SearchForUser);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Searching for user");
    }
}

void TaskModule::IdleStateUpdate() {
    if(false) {
        //TODO: przejście w stan rejestracji na podstawie danych z topica
        AwaitRegistration();
    }
    else if(logLevel <= Debug) {
        ROS_DEBUG("TaskModule: state == %d", state);
        ROS_DEBUG("TaskModule: Idle. CurrentUserXnId= %d", DataStorage::GetInstance().GetCurrentUserXnId());
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
            ROS_DEBUG("TaskModule: state == %d", state);
            ROS_DEBUG("TaskModule: Awaiting registration");
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
                    DataStorage::GetInstance().IsUserPose(DataStorage::GetInstance().GetCurrentUserXnId() - 1))) {
        //TODO: przejście w stan rejestracji na podstawie danych z topica
        AwaitRegistration();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        Search();
    }
    else {
        if(logLevel <= Debug) {
            ROS_DEBUG("TaskModule: state == %d", state);
            ROS_DEBUG("TaskModule: Following. CurrentUserXnId= %d", DataStorage::GetInstance().GetCurrentUserXnId());
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
            ROS_DEBUG("TaskModule: state == %d", state);
            ROS_DEBUG("TaskModule: Searching");
        }
    }
}