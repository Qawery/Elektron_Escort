#ifndef ELEKTRON_ESCORT_TASK_MODULE_H
#define ELEKTRON_ESCORT_TASK_MODULE_H

#define DEFAULT_TASK_MODULE_LOG_LEVEL Info
#define DEFAULT_WAIT_TIME_LIMIT 6.0
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
    Idle, Awaiting, Following, Waiting, Searching
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
    XnPoint3D lastUserPosition;
    double maxUserDistance;
    double timeElapsed;

    TaskModule() {}
    TaskModule(const TaskModule &);
    TaskModule &operator=(const TaskModule &);
    ~TaskModule() {}
    void BecomeIdle();
    void AwaitRegistration();
    void SaveTemplate();
    void ResumeFollowing();
    void WaitForReturn();
    void Search();
    void IdleStateUpdate();
    void AwaitingStateUpdate();
    void FollowingStateUpdate();
    void WaitingStateUpdate();
    void SearchingStateUpdate();
};

#endif //ELEKTRON_ESCORT_TASK_MODULE_H
