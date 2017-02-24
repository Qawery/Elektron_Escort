#ifndef ELEKTRON_ESCORT_SENSORS_MODULE_H
#define ELEKTRON_ESCORT_SENSORS_MODULE_H

#define DEFAULT_SENSORS_MODULE_LOG_LEVEL Debug
#define CALIBRATION_POSE "Psi"
#define CALIBRATION_SLOT 0
#define SMOOTHING_FACTOR 0.0f

#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "../Common.h"
#include "DataStorage.h"


enum SensorsState {
    Off, Calibrating, Working
};

class SensorsModule {
public:
    static SensorsModule& GetInstance() {
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
    void TurnSensorOff();
    void BeginCalibration();
    void ResetCalibration();
    void Work();

private:
    LogLevels logLevel;
    std::mutex stateMutex;
    xn::Context context;
    xn::UserGenerator userGenerator;
    SensorsState state;
    XnCallbackHandle userCallbacksHandle;
    XnCallbackHandle calibrationCallbacksHandle;
    XnCallbackHandle poseCallbacksHandle;

    SensorsModule() {}
    SensorsModule(const SensorsModule &);
    SensorsModule& operator=(const SensorsModule&);
    ~SensorsModule() {}

    //Callbacks
    static void User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_Exit(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_ReEnter(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie);
    static void UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie);
    static void UserCalibration_CalibrationComplete(xn::SkeletonCapability& skeleton, XnUserID userId, XnCalibrationStatus calibrationError, void* pCookie);
};

#endif //ELEKTRON_ESCORT_SENSORS_MODULE_H
