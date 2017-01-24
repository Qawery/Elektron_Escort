#include "SensorsModule.h"

SensorsModule::SensorsModule(DataStorage *_dataStorage)
{
    dataStorage = _dataStorage;
}

bool SensorsModule::Initialize()
{
    std::string configFilename = ros::package::getPath("elektron_escort") + "/OpenNi_config.xml";	//Pobranie nazwy pliku konfiguracyjnego.
    XnStatus result = context.InitFromXmlFile(configFilename.c_str());	//Initializacja z pliku konfiguracyjnego.
    if (result != XN_STATUS_OK)
    {
        ROS_ERROR("Initialization from Xml file failed: %s", xnGetStatusString(result));
        return false;
    }
    result = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);	//Próba znalezienia generatora obrazu głębi.
    if (result != XN_STATUS_OK)
    {
        ROS_ERROR("Find depth generator failed: %s", xnGetStatusString(result));
        return false;
    }
    result = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);	//Próba znalezienia genaratora użytkowników.
    if (result != XN_STATUS_OK)
    {
        result = userGenerator.Create(context);	//Nie znaleziono istniejącego generatora użytkowników, próbujemy utworzyć własny.
    }
    if (result != XN_STATUS_OK)
    {
        ROS_ERROR("Create user generator failed: %s", xnGetStatusString(result));
        return false;
    }
    if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
        ROS_INFO("User generator doesn't support skeleton");
        return false;
    }
    if (userGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
        isCalibrationPoseNeeded = TRUE;
        if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
        {
            ROS_INFO("Calibration pose required, but not supported");
            return false;
        }
        userGenerator.GetSkeletonCap().GetCalibrationPose(startingPose);
    }
    result = context.StartGeneratingAll();
    if (result != XN_STATUS_OK)
    {
        ROS_ERROR("Start generating all failed: %s", xnGetStatusString(result));
        return false;
    }
    userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    //userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, userCallbacksHandle);
    //userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, calibrationCallbacksHandle);
    userGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(this->UserPose_PoseDetected, NULL, NULL, poseCallbacksHandle);
    remove(DEFAULT_CALIBRATION_FILE_NAME);
    isCalibrationFilePresent = false;
    return true;
}

void SensorsModule::User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
}


void SensorsModule::User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
}


void SensorsModule::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie)
{
}


void SensorsModule::UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie)
{
}


void SensorsModule::UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID userId, XnBool success, void* cookie)
{
}

void SensorsModule::Update()
{
    context.WaitAndUpdateAll();
}

void SensorsModule::Finish()
{
    context.Release();
    if(isCalibrationFilePresent)
    {
        remove(DEFAULT_CALIBRATION_FILE_NAME);
    }
}