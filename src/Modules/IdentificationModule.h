#ifndef ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H
#define ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H

#define DEFAULT_IDENTIFICATION_MODULE_LOG_LEVEL Error
#define DEFAULT_IDENTIFICATION_THRESHOLD 1.0
#define DEFAULT_USER_ID_METHOD_TRUST 1.0
#define DEFAULT_HEIGHT_METHOD_TRUST 0.0

#include <ros/ros.h>
#include <ros/package.h>
#include "../Common.h"
#include "SensorsModule.h"
#include "IdentificationMethods/Identification_Method.h"
#include "IdentificationMethods/UserID_Method.h"
#include "IdentificationMethods/Height_Method.h"
#include "IdentificationMethods/Height_Method_Calib.h"


enum IdentificationStates {
    NoTemplate, PresentTemplate, SavingTemplate
};

enum ImplementedMethods {
    IM_UserId, IM_Height, IM_Height_Calib, IM_NUMBER_OF_METHODS
};

class IdentificationModule {
public:
    static IdentificationModule &GetInstance() {
        static IdentificationModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePrivate);
    void Update();
    void Finish();
    void ClearTemplate();
    void SaveTemplateOfCurrentUser();
    IdentificationStates GetState();

private:
    LogLevels logLevel;
    double identificationThreshold;
    IdentificationStates state;
    Identification_Method* methods[ImplementedMethods::IM_NUMBER_OF_METHODS];

    IdentificationModule() {}
    IdentificationModule(const IdentificationModule &);
    IdentificationModule &operator=(const IdentificationModule &);
    ~IdentificationModule() {}
    void ContinueSavingTemplate();
    void IdentifyUser();
    //DEBUG START
    int timer = 0;
    Height_Method_Calib* calib;
    //DEBUG END
};

#endif //ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H
