#ifndef ELEKTRON_ESCORT_SENSORSMODULE_H
#define ELEKTRON_ESCORT_SENSORSMODULE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "DefaultValues.h"
#include "DataStorage.h"

class SensorsModule
{
public:
    SensorsModule(DataStorage* _dataStorage);
    bool Initialize();
    void Update();
    void Finish();

private:
    DataStorage* dataStorage;
    xn::Context context;
    XnContext** contextPointer;
    xn::DepthGenerator depthGenerator;
    xn::UserGenerator userGenerator;
    XnCallbackHandle userCallbacksHandle;
    XnCallbackHandle calibrationCallbacksHandle;
    XnCallbackHandle poseCallbacksHandle;
    XnBool isCalibrationPoseNeeded;
    XnChar startingPose[20];
    bool isCalibrationFilePresent;

    static void User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie);
    static void UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie);
    static void UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie);
    static void UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID userId, XnBool success, void* cookie);
};

#endif //ELEKTRON_ESCORT_SENSORSMODULE_H
