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
        ROS_INFO("TaskModule: initialized");
    }
    AwaitRegistration();
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
void TaskModule::BecomeIdle() {
    state = Idle;
    MobilityModule::GetInstance().SetState(Stop);
    IdentificationModule::GetInstance().ClearTemplate();
    SensorsModule::GetInstance().ChangeStateTo(Off);
    DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
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

void TaskModule::SaveTemplate() {
    if(IdentificationModule::GetInstance().SaveTemplateOfCurrentUser()) {
        state = Following;
        MobilityModule::GetInstance().SetState(FollowUser);
        SensorsModule::GetInstance().ChangeStateTo(Working);
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Begin following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
        }
    }
    else {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Failed template creation for user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
        }
        DataStorage::GetInstance().SetCurrentUserXnId(NO_USER);
        SensorsModule::GetInstance().ClearCalibration();
    }

}

void TaskModule::ResumeFollowing() {
    state = Following;
    MobilityModule::GetInstance().SetState(FollowUser);
    if(logLevel <= Info) {
        ROS_INFO("TaskModule: Resuming following user: %d", DataStorage::GetInstance().GetCurrentUserXnId());
    }
}

void TaskModule::WaitForReturn() {
    state = Waiting;
    timeElapsed = 0.0;
    MobilityModule::GetInstance().SetState(Stop);
}

void TaskModule::Search() {
    state = Searching;
    timeElapsed = 0.0;
    if(lastUserPosition.X > 0) {
        MobilityModule::GetInstance().SetState(SearchForUserRight);
    }
    else {
        MobilityModule::GetInstance().SetState(SearchForUserLeft);
    }
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
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering awaiting from idle");
        }
        AwaitRegistration();
    }
}

void TaskModule::AwaitingStateUpdate() {
    if(false){
        //TODO: przejście w stan Idle na podstawie danych z topica
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering idle from awaiting");
        }
        BecomeIdle();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering template save state from awaiting");
        }
        SaveTemplate();
    }
}

void TaskModule::FollowingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering idle from following");
        }
        BecomeIdle();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER && DataStorage::GetInstance().IsUserPose(DataStorage::GetInstance().GetCurrentUserXnId() - 1)) {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering awaiting from following");
        }
        AwaitRegistration();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER) {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering waiting from following");
        }
        WaitForReturn();
    }
}

void TaskModule::WaitingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering idle from waiting");
        }
        BecomeIdle();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        ResumeFollowing();
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering following from waiting");
        }
    }
    else {
        if(lastUserPosition.Z > maxUserDistance) {
            if( timeElapsed >= waitTimeLimit) {
                Search();
                if(logLevel <= Info) {
                    ROS_INFO("TaskModule: Entering search from long waiting");
                }
            }
        }
        else {
            if( timeElapsed >= waitTimeLimit/2) {
                Search();
                if(logLevel <= Info) {
                    ROS_INFO("TaskModule: Entering search from short waiting");
                }
            }
        }
    }
}

void TaskModule::SearchingStateUpdate() {
    if(false) {
        //TODO: przejście w stan Idle na podstawie danych z topica
        BecomeIdle();
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering idle from search");
        }
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() == NO_USER && timeElapsed >= searchTimeLimit) {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering awaiting from search");
        }
        AwaitRegistration();
    }
    else if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        if(logLevel <= Info) {
            ROS_INFO("TaskModule: Entering following from search");
        }
        ResumeFollowing();
    }
}