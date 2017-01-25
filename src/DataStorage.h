#ifndef ELEKTRON_ESCORT_DATASTORAGE_H
#define ELEKTRON_ESCORT_DATASTORAGE_H

#include <ros/ros.h>
#include "DefaultValues.h"

class DataStorage
{
public:
    static DataStorage& GetInstance()
    {
        static DataStorage instance;
        return instance;
    }

    bool Initialize(ros::NodeHandle* nodeHandlePrivate);
    void UpdatePoseCooldowns(float timeElapsed);
    int GetMaxUsers();
    void SetMaxUsers(int _maxUsers);
    void SetPoseCooldownTime(float _poseCooldownTime);
    float GetPoseCooldownForUser(int _userNumber);

private:
    DataStorage() {}
    DataStorage(const DataStorage &);
    DataStorage& operator=(const DataStorage&);
    ~DataStorage() {}

    int maxUsers;
    float poseCooldownTime;
    std::vector<float> poseCooldownForUsers;
};

#endif //ELEKTRON_ESCORT_DATASTORAGE_H
