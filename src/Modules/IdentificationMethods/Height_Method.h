#ifndef ELEKTRON_ESCORT_HEIGHT_METHOD_H
#define ELEKTRON_ESCORT_HEIGHT_METHOD_H

#define DEFAULT_HEIGHT_TOLERANCE 30.0

#include <algorithm>
#include "Identification_Method.h"
#include "../SensorsModule.h"


class Height_Method : public Identification_Method {
public:
    void ClearTemplate();
    bool SaveTemplate();
    void Update();
    double RateUser(XnUserID userId);
    void LateUpdate();
    double CalculateHeight(XnUserID const& userId);
    double CalculateHeight(XnUserID const& userId, double& confidence);

private:
    double originalHeight;
    double CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB, double& confidence);
};

#endif //ELEKTRON_ESCORT_HEIGHT_METHOD_H
