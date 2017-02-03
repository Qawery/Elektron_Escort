#ifndef ELEKTRON_ESCORT_TASK_MODULE_H
#define ELEKTRON_ESCORT_TASK_MODULE_H

#define DEFAULT_TASK_MODULE_LOG_LEVEL Info

#include <ros/ros.h>
#include <ros/package.h>
#include <XnCppWrapper.h>
#include "../Common.h"
#include "SensorsModule.h"
#include "IdentificationModule.h"
#include "DataStorage.h"


enum TaskState
{
    Idle, Awaiting, Following, Searching
};

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
    //...

private:
    //System fields
    LogLevels logLevel;

    //Task fields
    TaskState state;

    //System functions
    TaskModule() {}
    TaskModule(const TaskModule &);
    TaskModule &operator=(const TaskModule &);
    ~TaskModule() {}

    //Task functions
    void BecomeIdle();
    void AwaitRegistration();
    void BeginFollowing();
    void ResumeFollowing();
    void Search();

    void IdleStateUpdate();
    void AwaitingStateUpdate();
    void FollowingStateUpdate();
    void SearchingStateUpdate();
};

#endif //ELEKTRON_ESCORT_TASK_MODULE_H
