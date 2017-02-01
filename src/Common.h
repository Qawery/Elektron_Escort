#ifndef ELEKTRON_ESCORT_COMMON_H
#define ELEKTRON_ESCORT_COMMON_H

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

#endif //ELEKTRON_ESCORT_COMMON_H
