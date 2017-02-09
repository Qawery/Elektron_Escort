#include "SensorsModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SensorsModule::Initialize(ros::NodeHandle* nodeHandlePrivate) {
    int _logLevel;
    if(!nodeHandlePrivate->getParam("sensorsModuleLogLevel", _logLevel)) {
        ROS_WARN("SensorsModule: Log level not found, using default");
        logLevel = DEFAULT_SENSORS_MODULE_LOG_LEVEL;
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
                ROS_WARN("SensorsModule: Requested invalid log level, using default");
                logLevel = DEFAULT_SENSORS_MODULE_LOG_LEVEL;
                break;
        }
    }
    XnStatus result = context.Init();
    if (result != XN_STATUS_OK) {
        if(logLevel <= Error) {
            ROS_ERROR("SensorsModule: Initialization from Xml file failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    result = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);	//Próba znalezienia genaratora użytkowników.
    if (result != XN_STATUS_OK) {
        result = userGenerator.Create(context);	//Nie znaleziono istniejącego generatora użytkowników, próbujemy utworzyć własny.
    }
    if (result != XN_STATUS_OK) {
        if(logLevel <= Error) {
            ROS_ERROR("SensorsModule: Create user generator failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        if(logLevel <= Error) {
            ROS_ERROR("SensorsModule: User generator doesn't support skeleton");
        }
        return false;
    }
    if (userGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            if(logLevel <= Error) {
                ROS_ERROR("SensorsModule: Calibration pose required, but not supported");
            }
            return false;
        }
    }
    result = context.StartGeneratingAll();
    if (result != XN_STATUS_OK) {
        if(logLevel <= Error) {
            ROS_ERROR("SensorsModule: Start generating all failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    state = Off;
    stateMutex.lock();
    userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, userCallbacksHandle);
    userGenerator.RegisterToUserExit(User_Exit, NULL, userCallbacksHandle);
    userGenerator.RegisterToUserReEnter(User_ReEnter, NULL, userCallbacksHandle);
    userGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, calibrationCallbacksHandle);
    userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, calibrationCallbacksHandle);
    userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, poseCallbacksHandle);
    stateMutex.unlock();
    if(logLevel <= Info) {
        ROS_INFO("SensorsModule: initialized");
    }
    return true;
}

void SensorsModule::Update() {
    context.WaitAnyUpdateAll();
}

void SensorsModule::Finish() {
    context.Release();
}

LogLevels SensorsModule::GetLogLevel() {
    return logLevel;
}

xn::UserGenerator SensorsModule::GetUserGenerator() {
    return userGenerator;
}

void SensorsModule::LockStateMutex() {
    stateMutex.lock();
}

void SensorsModule::UnlockStateMutex() {
    stateMutex.unlock();
}

SensorsState SensorsModule::GetState() {
    return state;
}

void SensorsModule::ChangeStateTo(SensorsState newState) {
    stateMutex.lock();
    if(state == newState) {
        if(logLevel <= Warn) {
            ROS_WARN("SensorsModule: Attempted state change to same state: %d", state);
        }
    }
    ExitState(state);
    EnterState(newState);
    if(logLevel <= Info) {
        ROS_INFO("SensorsModule: State change to: %d", state);
    }
    stateMutex.unlock();
}

void SensorsModule::ClearCalibration() {
    stateMutex.lock();
    userGenerator.GetSkeletonCap().ClearCalibrationData(CALIBRATION_SLOT);
    stateMutex.unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorsModule::ExitState(SensorsState state) {
    XnUInt16 numberOfUsers = userGenerator.GetNumberOfUsers();
    XnUserID userIds[numberOfUsers];
    userGenerator.GetUsers(userIds, numberOfUsers);
    switch (state) {
        case Off:
            break;
        case Calibrating:
            for(int i=0; i < numberOfUsers; ++i) {
                userGenerator.GetPoseDetectionCap().StopPoseDetection(userIds[i]);
                userGenerator.GetSkeletonCap().AbortCalibration(userIds[i]);
            }
            break;
        case Working:
            for(int i=0; i < numberOfUsers; ++i) {
                userGenerator.GetPoseDetectionCap().StopPoseDetection(userIds[i]);
                userGenerator.GetSkeletonCap().StopTracking(userIds[i]);
            }
            userGenerator.GetSkeletonCap().ClearCalibrationData(CALIBRATION_SLOT);
            break;
    }
}

void SensorsModule::EnterState(SensorsState newState) {
    XnUInt16 numberOfUsers = userGenerator.GetNumberOfUsers();
    XnUserID userIds[numberOfUsers];
    userGenerator.GetUsers(userIds, numberOfUsers);
    switch (newState) {
        case Off:
            break;
        case Calibrating:
            for(int i=0; i < numberOfUsers; ++i) {
                userGenerator.GetPoseDetectionCap().StartPoseDetection(CALIBRATION_POSE, userIds[i]);
            }
            break;
        case Working:
            if(!userGenerator.GetSkeletonCap().IsCalibrationData(CALIBRATION_SLOT)) {
                if(SensorsModule::GetInstance().GetLogLevel() <= Error) {
                    ROS_ERROR("SensorsModule: no calibration data present");
                    EnterState(Off);
                    return;
                }
            }
            else {
                for (int i = 0; i < numberOfUsers; ++i) {
                    userGenerator.GetPoseDetectionCap().StartPoseDetection(CALIBRATION_POSE, userIds[i]);
                    userGenerator.GetSkeletonCap().LoadCalibrationData(userIds[i], CALIBRATION_SLOT);
                    userGenerator.GetSkeletonCap().StartTracking(userIds[i]);
                }
            }
            break;
    }
    state = newState;
}

void SensorsModule::LoadCalibrationDataForUser(XnUserID userId) {
    stateMutex.lock();
    if(userGenerator.GetSkeletonCap().IsCalibrationData(CALIBRATION_SLOT)) {
        userGenerator.GetSkeletonCap().LoadCalibrationData(userId, CALIBRATION_SLOT);
        userGenerator.GetSkeletonCap().StartTracking(userId);
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
            ROS_DEBUG("SensorsModule: User: %d- loaded calibration data, tracking", userId);
        }
    }
    else {
        if(SensorsModule::GetInstance().GetLogLevel() <= Error) {
            ROS_ERROR("SensorsModule: User: %d- missing calibration data", userId);
        }
    }
    stateMutex.unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Callbacks
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorsModule::User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie) {
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
        ROS_DEBUG("SensorsModule: User: %d- new", userId);
    }
    if (SensorsModule::GetInstance().GetState() != Off) {
        generator.GetPoseDetectionCap().StartPoseDetection(CALIBRATION_POSE, userId);
        if(SensorsModule::GetInstance().GetState() == Working) {
            SensorsModule::GetInstance().LoadCalibrationDataForUser(userId);
        }
    }
    DataStorage::GetInstance().UserNew(userId);
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::User_Exit(xn::UserGenerator &generator, XnUserID userId, void *cookie) {
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
        ROS_DEBUG("SensorsModule: User: %d- exit", userId);
    }
    DataStorage::GetInstance().UserExit(userId);
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::User_ReEnter(xn::UserGenerator &generator, XnUserID userId, void *cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
        ROS_DEBUG("SensorsModule: User: %d- reenter", userId);
    }
    DataStorage::GetInstance().UserReEnter(userId);
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
        ROS_DEBUG("SensorsModule: User: %d- lost", userId);
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
        ROS_DEBUG("SensorsModule: User: %d- pose detected", userId);
    }
    if (SensorsModule::GetInstance().GetState() == Calibrating) {
        if(DataStorage::GetInstance().IsPoseCooldownPassed(userId-1)) {
            SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().RequestCalibration(userId, TRUE);
        }
    }
    DataStorage::GetInstance().UserPose(userId - 1);
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::UserCalibration_CalibrationStart(xn::SkeletonCapability& skeleton, XnUserID userId, void* cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
        ROS_DEBUG("SensorsModule: User: %d- calibration start", userId);
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::UserCalibration_CalibrationComplete(xn::SkeletonCapability& skeleton, XnUserID userId, XnCalibrationStatus calibrationError, void* pCookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(calibrationError == XN_CALIBRATION_STATUS_OK) {
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
            ROS_DEBUG("SensorsModule: User: %d- calibration successful", userId);
        }
        if(SensorsModule::GetInstance().GetState() == Calibrating && !SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().IsCalibrationData(CALIBRATION_SLOT)) {
            if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
                ROS_DEBUG("SensorsModule: User: %d- saved calibration data", userId);
            }
            SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().SaveCalibrationData(userId, CALIBRATION_SLOT);
            DataStorage::GetInstance().SetCurrentUserXnId(userId);
        }
    }
    else {
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug) {
            ROS_DEBUG("SensorsModule: User: %d- calibration failed", userId);
        }
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}
