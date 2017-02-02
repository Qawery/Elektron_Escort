#ifndef ELEKTRON_ESCORT_COMMON_H
#define ELEKTRON_ESCORT_COMMON_H

#define NO_USER -1

enum LogLevels
{
    Debug, Info, Warn, Error
};

enum TaskState
{
    Idle, Awaiting, Following, Searching
};

enum SensorsState
{
    Off, Calibrating, Working
};

enum DrivesState
{
    Stop, FollowUser, SearchForUser
};

#endif //ELEKTRON_ESCORT_COMMON_H
