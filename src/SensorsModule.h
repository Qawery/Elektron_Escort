#ifndef ELEKTRON_ESCORT_SENSORSMODULE_H
#define ELEKTRON_ESCORT_SENSORSMODULE_H

#define DEFAULT_SENSORS_MODULE_LOG_LEVEL Debug
#define CALIBRATION_POSE "psi"
#define CALIBRATION_SLOT 0

#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "Common.h"
#include "DataStorage.h"
#include "TaskModule.h"


class SensorsModule
{
public:
    //System functions
    static SensorsModule& GetInstance()
    {
        static SensorsModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void Update();
    void Finish();
    LogLevels GetLogLevel();
    xn::UserGenerator GetUserGenerator();
    void LockStateMutex();
    void UnlockStateMutex();
    SensorsState GetState();

    //Task functions
    void ChangeStateTo(SensorsState newState);
    void UnsafeChangeStateTo(SensorsState newState);
    void PoseDetected(XnUserID userId);

private:
    //System fields
    LogLevels logLevel;
    std::mutex stateMutex;
    std::mutex newDataMutex;
    xn::Context context;
    xn::UserGenerator userGenerator;
    SensorsState state;
    XnCallbackHandle userCallbacksHandle;
    XnCallbackHandle calibrationCallbacksHandle;
    XnCallbackHandle poseCallbacksHandle;

    //Task fields
    std::vector<bool> newPoseDetected;

    //System functions
    SensorsModule() {}
    SensorsModule(const SensorsModule &);
    SensorsModule& operator=(const SensorsModule&);
    ~SensorsModule() {}

    //Task functions
    void ExitState(SensorsState state);
    void EnterState(SensorsState state);
    void SendNewDataToStorage();
    void LoadCalibrationDataForUser(XnUserID userId);

    //Callbacks
    static void User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_Exit(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_ReEnter(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie);
    static void UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie);
    static void UserCalibration_CalibrationComplete(xn::SkeletonCapability& skeleton, XnUserID userId, XnCalibrationStatus calibrationError, void* pCookie);
};
#endif //ELEKTRON_ESCORT_SENSORSMODULE_H
