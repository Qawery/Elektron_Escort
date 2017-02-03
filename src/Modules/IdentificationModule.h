#ifndef ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H
#define ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H

#define DEFAULT_IDENTIFICATION_MODULE_LOG_LEVEL Info
#define DEFAULT_IDENTIFICATION_THRESHOLD 0.5f
#define DEFAULT_USER_ID_METHOD_TRUST 1.0f

#include <ros/ros.h>
#include <ros/package.h>
#include "../Common.h"
#include "SensorsModule.h"
#include "IdentificationMethods/IdentificationMethod.h"
#include "IdentificationMethods/UserID_Method.h"


class IdentificationModule {
public:
    //System functions
    static IdentificationModule &GetInstance() {
        static IdentificationModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePrivate);
    void Update();

    //Task functions
    void ClearTemplate();
    bool SaveTemplateOfCurrentUser();

private:
    //System fields
    LogLevels logLevel;
    float identificationThreshold;
    bool isTemplateSaved;

    //Task fields
    UserID_Method userID_method;

    //System functions
    IdentificationModule() {}
    IdentificationModule(const IdentificationModule &);
    IdentificationModule &operator=(const IdentificationModule &);
    ~IdentificationModule() {}

    //Task functions
    //...
};

#endif //ELEKTRON_ESCORT_IDENTIFICATION_MODULE_H
