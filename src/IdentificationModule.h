#ifndef ELEKTRON_ESCORT_IDENTIFICATIONMODULE_H
#define ELEKTRON_ESCORT_IDENTIFICATIONMODULE_H

#define DEFAULT_IDENTIFICATION_MODULE_LOG_LEVEL Debug

#include <ros/ros.h>
#include <ros/package.h>
#include "Common.h"


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
    //...

private:
    //System fields
    LogLevels logLevel;

    //Task fields
    //...

    //System functions
    IdentificationModule() {}
    IdentificationModule(const IdentificationModule &);
    IdentificationModule &operator=(const IdentificationModule &);
    ~IdentificationModule() {}

    //Task functions
    //...
};
#endif //ELEKTRON_ESCORT_IDENTIFICATIONMODULE_H
