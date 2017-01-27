#ifndef ELEKTRON_ESCORT_SENSORSMODULE_H
#define ELEKTRON_ESCORT_SENSORSMODULE_H

#define DEFAULT_SENSORS_MODULE_LOG_LEVEL Debug
#define CALIBRATION_FILE_NAME "calibration.bin"

#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "Common.h"
#include "DataStorage.h"

class SensorsModule
{
public:
    static SensorsModule& GetInstance()
    {
        static SensorsModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void Update();
    void Finish();
    void LockCalibrationMutex();
    void UnLockCalibrationMutex();
    void LockNewDataMutex();
    void UnLockNewDataMutex();
    void SaveCalibrationFile(XnUserID userId);
    void LoadCalibrationFromFile(xn::UserGenerator& generator, XnUserID userId);
    void RemoveCalibrationFile();
    bool GetIsCalibrationFilePresent();
    xn::UserGenerator GetUserGenerator();
    void PoseDetected(XnUserID userId);
    LogLevels GetLogLevel();

private:
    LogLevels logLevel;
    std::vector<bool> newPoseDetected;
    bool isCalibrationFilePresent;
    std::mutex calibrationMutex;
    std::mutex newDataMutex;
    xn::Context context;
    xn::DepthGenerator depthGenerator;
    xn::UserGenerator userGenerator;
    XnCallbackHandle userCallbacksHandle;
    XnCallbackHandle calibrationCallbacksHandle;
    XnCallbackHandle poseCallbacksHandle;
    XnChar startingPose[20];

    SensorsModule() {}
    SensorsModule(const SensorsModule &);
    SensorsModule& operator=(const SensorsModule&);
    ~SensorsModule() {}
    void SendNewDataToStorage();

    static void User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_Exit(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_ReEnter(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie);
    static void UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie);
    static void UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID userId, XnBool success, void* cookie);
};

#endif //ELEKTRON_ESCORT_SENSORSMODULE_H
