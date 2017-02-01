#ifndef ELEKTRON_ESCORT_TASKMODULE_H
#define ELEKTRON_ESCORT_TASKMODULE_H

#define DEFAULT_TASK_MODULE_LOG_LEVEL Debug

#include <ros/ros.h>
#include <ros/package.h>
#include <XnCppWrapper.h>
#include "Common.h"
#include "DataStorage.h"


class TaskModule {
public:
    //System functions
    static TaskModule &GetInstance() {
        static TaskModule instance;
        return instance;
    }

    bool Initialize(ros::NodeHandle *nodeHandlePrivate);
    void Update();

    //Task functions
    void CalibrationCompleted(XnUserID newUserId);

private:
    //System fields
    LogLevels logLevel;

    //Task fields

    //System functions
    TaskModule() {}
    TaskModule(const TaskModule &);
    TaskModule &operator=(const TaskModule &);
    ~TaskModule() {}

    //Task functions
};

#endif //ELEKTRON_ESCORT_TASKMODULE_H
