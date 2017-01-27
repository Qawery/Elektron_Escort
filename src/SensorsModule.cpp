#include "SensorsModule.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public methods
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
    std::string configFilename = ros::package::getPath("elektron_escort") + "/OpenNi_config.xml";	//Pobranie nazwy pliku konfiguracyjnego.
    XnStatus result = context.InitFromXmlFile(configFilename.c_str());	//Initializacja z pliku konfiguracyjnego.
    if (result != XN_STATUS_OK)
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Initialization from Xml file failed: %s", xnGetStatusString(result));
        }
        return false;
    }
    result = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);	//Próba znalezienia generatora obrazu głębi.
    if (result != XN_STATUS_OK)
    {
        if(logLevel <= Error)
        {
            ROS_ERROR("Find depth generator failed: %s", xnGetStatusString(result));
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
        userGenerator.GetSkeletonCap().GetCalibrationPose(startingPose);
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
    userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, userCallbacksHandle);
    userGenerator.RegisterToUserExit(User_Exit, NULL, userCallbacksHandle);
    userGenerator.RegisterToUserReEnter(User_ReEnter, NULL, userCallbacksHandle);
    userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, calibrationCallbacksHandle);
    userGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, poseCallbacksHandle);
    calibrationMutex.lock();
    RemoveCalibrationFile();
    calibrationMutex.unlock();
    newPoseDetected.resize(DataStorage::GetInstance().GetMaxUsers());
    for(int i=0; i<DataStorage::GetInstance().GetMaxUsers(); ++i)
    {
        newPoseDetected.push_back(false);
    }
    return true;
}

void SensorsModule::Update()
{
    context.WaitAndUpdateAll();
    SendNewDataToStorage();
}

void SensorsModule::Finish()
{
    context.Release();
    calibrationMutex.lock();
    if(isCalibrationFilePresent)
    {
        remove(CALIBRATION_FILE_NAME);
        isCalibrationFilePresent = false;
    }
    calibrationMutex.unlock();
}

void SensorsModule::LockCalibrationMutex()
{
    calibrationMutex.lock();
}

void SensorsModule::UnLockCalibrationMutex()
{
    calibrationMutex.unlock();
}

void SensorsModule::LockNewDataMutex()
{
    newDataMutex.lock();
}

void SensorsModule::UnLockNewDataMutex()
{
    newDataMutex.unlock();
}

void SensorsModule::SaveCalibrationFile(XnUserID userId)
{
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().SaveCalibrationDataToFile(userId, CALIBRATION_FILE_NAME);
    isCalibrationFilePresent = true;
}

void SensorsModule::LoadCalibrationFromFile(xn::UserGenerator& generator, XnUserID userId)
{
    if(isCalibrationFilePresent)
    {
        generator.GetSkeletonCap().LoadCalibrationDataFromFile(userId, CALIBRATION_FILE_NAME);
    }
}

void SensorsModule::RemoveCalibrationFile()
{
    remove(CALIBRATION_FILE_NAME);
    isCalibrationFilePresent = false;
}

bool SensorsModule::GetIsCalibrationFilePresent()
{
    return isCalibrationFilePresent;
}

xn::UserGenerator SensorsModule::GetUserGenerator()
{
    return userGenerator;
}

void SensorsModule::PoseDetected(XnUserID userId)
{
    newPoseDetected[userId-1] = true;
}

LogLevels SensorsModule::GetLogLevel()
{
    return logLevel;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private methods
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void SensorsModule::User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- new", userId);
    }
    generator.GetPoseDetectionCap().StartPoseDetection("Psi", userId);
    SensorsModule::GetInstance().LockCalibrationMutex();
    if(SensorsModule::GetInstance().GetIsCalibrationFilePresent())
    {
        SensorsModule::GetInstance().LoadCalibrationFromFile(generator, userId);
        generator.GetSkeletonCap().StartTracking(userId);
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
        {
            ROS_DEBUG("User: %d- loaded calibration data from file");
        }
    }
    SensorsModule::GetInstance().UnLockCalibrationMutex();
}

void SensorsModule::User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- lost", userId);
    }
}

void SensorsModule::User_Exit(xn::UserGenerator &generator, XnUserID userId, void *cookie)
{
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- exit", userId);
    }
}

void SensorsModule::User_ReEnter(xn::UserGenerator &generator, XnUserID userId, void *cookie)
{
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- reenter", userId);
    }
}

void SensorsModule::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie)
{
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- pose detected", userId);
    }
    SensorsModule::GetInstance().LockCalibrationMutex();
    if(!SensorsModule::GetInstance().GetIsCalibrationFilePresent())
    {
        SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().RequestCalibration(userId, TRUE);
    }
    SensorsModule::GetInstance().UnLockCalibrationMutex();
    SensorsModule::GetInstance().LockNewDataMutex();
    SensorsModule::GetInstance().PoseDetected(userId);
    SensorsModule::GetInstance().UnLockNewDataMutex();
}

void SensorsModule::UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie)
{
    if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
    {
        ROS_DEBUG("User: %d- calibration start", userId);
    }
}

void SensorsModule::UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID userId, XnBool success, void* cookie)
{
    if(success)
    {
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
        {
            ROS_DEBUG("User: %d- calibration successful", userId);
        }
        SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().StartTracking(userId);
        SensorsModule::GetInstance().LockCalibrationMutex();
        if(!SensorsModule::GetInstance().GetIsCalibrationFilePresent())
        {
            SensorsModule::GetInstance().SaveCalibrationFile(userId);
            if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
            {
                ROS_DEBUG("User: %d- saved calibration file", userId);
            }
            //TODO: rozpoczęcie śledzenia
        }
        SensorsModule::GetInstance().UnLockCalibrationMutex();
    }
    else
    {
        if(SensorsModule::GetInstance().GetLogLevel() <= Debug)
        {
            ROS_DEBUG("User: %d- calibration failed", userId);
        }
    }
}
