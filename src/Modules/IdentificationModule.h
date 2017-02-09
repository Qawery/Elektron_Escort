#ifndef ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H
#define ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H

#define DEFAULT_IDENTIFICATION_MODULE_LOG_LEVEL Info
#define DEFAULT_IDENTIFICATION_THRESHOLD 0.5
#define DEFAULT_USER_ID_METHOD_TRUST 1.0

#include <ros/ros.h>
#include <ros/package.h>
#include "../Common.h"
#include "SensorsModule.h"
#include "IdentificationMethods/IdentificationMethod.h"
#include "IdentificationMethods/UserID_Method.h"

enum IdentificationStates {
    NoTemplate, PresentTemplate
};

class IdentificationModule {
public:
    static IdentificationModule &GetInstance() {
        static IdentificationModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePrivate);
    void Update();
    void ClearTemplate();
    bool SaveTemplateOfCurrentUser();

private:
    LogLevels logLevel;
    double identificationThreshold;
    IdentificationStates state;
    UserID_Method userID_method;

    IdentificationModule() {}
    IdentificationModule(const IdentificationModule &);
    IdentificationModule &operator=(const IdentificationModule &);
    ~IdentificationModule() {}
};

#endif //ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H
