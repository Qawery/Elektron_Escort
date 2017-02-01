#ifndef ELEKTRON_ESCORT_MOBILITYMODULE_H
#define ELEKTRON_ESCORT_MOBILITYMODULE_H

#define DEFAULT_MOBILITY_MODULE_LOG_LEVEL Debug

#include <ros/ros.h>
#include <ros/package.h>
#include "Common.h"


class MobilityModule {
public:
    //System functions
    static MobilityModule &GetInstance() {
        static MobilityModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePrivate);
    void Update();

    //Task functions
    void SetState(DrivesState newState);

private:
    //System fields
    LogLevels logLevel;

    //Task fields
    DrivesState state;

    //System functions
    MobilityModule() {}
    MobilityModule(const MobilityModule &);
    MobilityModule &operator=(const MobilityModule &);
    ~MobilityModule() {}

    //Task functions
};

#endif //ELEKTRON_ESCORT_MOBILITYMODULE_H
