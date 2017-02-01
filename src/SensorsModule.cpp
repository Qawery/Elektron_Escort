#include "SensorsModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//System functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SensorsModule::Initialize(ros::NodeHandle* nodeHandlePrivate)
{
    int _logLevel;
    if(!nodeHandlePrivate->getParam("sensorsModuleLogLevel", _logLevel))
    {
        ROS_WARN("sensorsModuleLogLevel not found, using default");
        logLevel = DEFAULT_SENSORS_MODULE_LOG_LEVEL;
    }
    else
    {
        switch (_logLevel)
        {
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
                ROS_WARN("Requested invalid sensorsModuleLogLevel, using default");
                logLevel = DEFAULT_SENSORS_MODULE_LOG_LEVEL;
                break;
        }
    }
    XnStatus result = context.Init();
    if (result != XN_STATUS_OK)
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Initialization from Xml file failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    result = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);	//Próba znalezienia genaratora użytkowników.
    if (result != XN_STATUS_OK)
    {
        result = userGenerator.Create(context);	//Nie znaleziono istniejącego generatora użytkowników, próbujemy utworzyć własny.
    }
    if (result != XN_STATUS_OK)
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Create user generator failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("User generator doesn't support skeleton");
        }
        return false;
    }
    if (userGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
        if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
        {
            if(logLevel <= Error)
            {
                ROS_ERROR("Calibration pose required, but not supported");
            }
            return false;
        }
    }
    result = context.StartGeneratingAll();
    if (result != XN_STATUS_OK)
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Start generating all failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    state = Off;
    newPoseDetected.resize(DataStorage::GetInstance().GetMaxUsers());
    for(int i=0; i<DataStorage::GetInstance().GetMaxUsers(); ++i)
    {
        newPoseDetected.push_back(false);
    }
    newDataMutex.lock();
    stateMutex.lock();
    userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, userCallbacksHandle);
    userGenerator.RegisterToUserExit(User_Exit, NULL, userCallbacksHandle);
    userGenerator.RegisterToUserReEnter(User_ReEnter, NULL, userCallbacksHandle);
    userGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, calibrationCallbacksHandle);
    userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, calibrationCallbacksHandle);
    userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, poseCallbacksHandle);
    stateMutex.unlock();
    newDataMutex.unlock();
    return true;
}

void SensorsModule::Update()
{
    context.WaitAndUpdateAll();
    SendNewDataToStorage();
    XnUInt16 numberOfUsers = userGenerator.GetNumberOfUsers();
    XnUserID userIds[numberOfUsers];
    userGenerator.GetUsers(userIds, numberOfUsers);
    for(int i=0; i < numberOfUsers; ++i)
    {
        //TODO: wyślij dane o użytkownikach do DataStorage
    }
}

void SensorsModule::Finish()
{
    context.Release();
}

LogLevels SensorsModule::GetLogLevel()
{
    return logLevel;
}

xn::UserGenerator SensorsModule::GetUserGenerator()
{
    return userGenerator;
}

void SensorsModule::LockStateMutex()
{
    stateMutex.lock();
}

void SensorsModule::UnlockStateMutex()
{
    stateMutex.unlock();
}

SensorsState SensorsModule::GetState()
{
    return state;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Task functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorsModule::ChangeStateTo(SensorsState newState)
{
    stateMutex.lock();
    ExitState(state);
    EnterState(newState);
    state = newState;
    stateMutex.unlock();
}

void SensorsModule::UnsafeChangeStateTo(SensorsState newState)
{
    ExitState(state);
    EnterState(newState);
    state = newState;
}

void SensorsModule::PoseDetected(XnUserID userId)
{
    newDataMutex.lock();
    newPoseDetected[userId-1] = true;
    newDataMutex.unlock();
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
void SensorsModule::ExitState(SensorsState state)
{
    XnUInt16 numberOfUsers = userGenerator.GetNumberOfUsers();
    XnUserID userIds[numberOfUsers];
    userGenerator.GetUsers(userIds, numberOfUsers);
    switch (state)
    {
        case Off:
            break;

        case Calibrating:
            for(int i=0; i < numberOfUsers; ++i)
            {
                userGenerator.GetPoseDetectionCap().StopPoseDetection(userIds[i]);
                userGenerator.GetSkeletonCap().AbortCalibration(userIds[i]);
            }
            break;

        case Working:
            for(int i=0; i < numberOfUsers; ++i)
            {
                userGenerator.GetPoseDetectionCap().StopPoseDetection(userIds[i]);
                userGenerator.GetSkeletonCap().StopTracking(userIds[i]);
            }
            break;
    }
}

void SensorsModule::EnterState(SensorsState state)
{
    XnUInt16 numberOfUsers = userGenerator.GetNumberOfUsers();
    XnUserID userIds[numberOfUsers];
    userGenerator.GetUsers(userIds, numberOfUsers);
    switch (state)
    {
        case Off:
            break;

        case Calibrating:
            for(int i=0; i < numberOfUsers; ++i)
            {
                userGenerator.GetPoseDetectionCap().StartPoseDetection("psi", userIds[i]);
            }
            break;

        case Working:
            for(int i=0; i < numberOfUsers; ++i)
            {
                userGenerator.GetPoseDetectionCap().StartPoseDetection("psi", userIds[i]);
                userGenerator.GetSkeletonCap().LoadCalibrationData(userIds[i], CALIBRATION_SLOT);
                userGenerator.GetSkeletonCap().StartTracking(userIds[i]);
            }
            break;
    }
}

void SensorsModule::SendNewDataToStorage()
{
    newDataMutex.lock();
    for(int i=0; i<DataStorage::GetInstance().GetMaxUsers(); ++i)
    {
        if(newPoseDetected[i])
        {
            DataStorage::GetInstance().PoseDetectedForUser(i);
        }
        newPoseDetected[i] = false;
    }
    newDataMutex.unlock();
}

void SensorsModule::LoadCalibrationDataForUser(XnUserID userId)
{
    stateMutex.lock();
    if(userGenerator.GetSkeletonCap().IsCalibrationData(CALIBRATION_SLOT))
    {
        userGenerator.GetSkeletonCap().LoadCalibrationData(userId, CALIBRATION_SLOT);
        userGenerator.GetSkeletonCap().StartTracking(userId);
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
        {
            ROS_DEBUG("User: %d- loaded calibration data, tracking", userId);
        }
    }
    else
    {
        if(SensorsModule::GetInstance().GetLogLevel() <= Error)
        {
            ROS_ERROR("User: %d- missing calibration data", userId);
        }
    }
    stateMutex.unlock();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Callbacks
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void SensorsModule::User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- new", userId);
    }
    if (SensorsModule::GetInstance().GetState() != Off)
    {
        generator.GetPoseDetectionCap().StartPoseDetection("Psi", userId);
        if(SensorsModule::GetInstance().GetState() == Working)
        {
            SensorsModule::GetInstance().LoadCalibrationDataForUser(userId);
        }
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::User_Exit(xn::UserGenerator &generator, XnUserID userId, void *cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- exit", userId);
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::User_ReEnter(xn::UserGenerator &generator, XnUserID userId, void *cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- reenter", userId);
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- lost", userId);
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- pose detected", userId);
    }
    SensorsModule::GetInstance().PoseDetected(userId);
    if (SensorsModule::GetInstance().GetState() == Calibrating)
    {
        if(!SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().IsCalibrating(userId))
        {
            SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().RequestCalibration(userId, TRUE);
        }
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::UserCalibration_CalibrationStart(xn::SkeletonCapability& skeleton, XnUserID userId, void* cookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- calibration start", userId);
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}

void SensorsModule::UserCalibration_CalibrationComplete(xn::SkeletonCapability& skeleton, XnUserID userId, XnCalibrationStatus calibrationError, void* pCookie)
{
    SensorsModule::GetInstance().LockStateMutex();
    if(calibrationError == XN_CALIBRATION_STATUS_OK)
    {
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
        {
            ROS_DEBUG("User: %d- calibration successful", userId);
        }
        if(SensorsModule::GetInstance().GetState() == Calibrating)
        {
            if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
            {
                ROS_DEBUG("User: %d- saved calibration data", userId);
            }
            SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().SaveCalibrationData(userId, CALIBRATION_SLOT);
            SensorsModule::GetInstance().UnsafeChangeStateTo(Working);
            TaskModule::GetInstance().CalibrationCompleted(userId);
        }
    }
    else
    {
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
        {
            ROS_DEBUG("User: %d- calibration failed", userId);
        }
    }
    SensorsModule::GetInstance().UnlockStateMutex();
}
