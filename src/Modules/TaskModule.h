#ifndef ELEKTRON_ESCORT_TASK_MODULE_H
#define ELEKTRON_ESCORT_TASK_MODULE_H

#define DEFAULT_TASK_MODULE_LOG_LEVEL Info
#define DEFAULT_WAIT_TIME_LIMIT 5.0
#define DEFAULT_SEARCH_TIME_LIMIT 10.0
#define DEFAULT_MAX_USER_DISTANCE 3000.0

#include <ros/ros.h>
#include <ros/package.h>
#include <XnCppWrapper.h>
#include "../Common.h"
#include "SensorsModule.h"
#include "IdentificationModule.h"
#include "DataStorage.h"


enum TaskState
{
    Idle, Awaiting, Saving, Following, Waiting, Searching
};

class TaskModule {
public:
    static TaskModule &GetInstance() {
        static TaskModule instance;
        return instance;
    }
    bool Initialize(ros::NodeHandle *nodeHandlePrivate);
    void Update(double _timeElapsed);

private:
    LogLevels logLevel;
    TaskState state;
    double waitTimeLimit;
    double searchTimeLimit;
    double maxUserDistance;
    double timeElapsed;

    TaskModule() {}
    TaskModule(const TaskModule &);
    TaskModule &operator=(const TaskModule &);
    ~TaskModule() {}
    void IdleStateEnter();
    void AwaitingStateEnter();
    void SavingStateEnter();
    void FollowingStateEnter();
    void WaitingStateEnter();
    void SearchingStateEnter();

    void IdleStateUpdate();
    void AwaitingStateUpdate();
    void SavingTemplateUpdate();
    void FollowingStateUpdate();
    void WaitingStateUpdate();
    void SearchingStateUpdate();
};

#endif //ELEKTRON_ESCORT_TASK_MODULE_H
