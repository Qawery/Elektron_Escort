#ifndef ELEKTRON_ESCORT_HEIGHT_METHOD_H
#define ELEKTRON_ESCORT_HEIGHT_METHOD_H

#define DEFAULT_NUMBER_OF_TEMPLATE_SAMPLES 60
#define MAX_NUMBER_OF_SAMPLES 30
#define DEFAULT_HEIGHT_TOLERANCE 20.0
#define DEFAULT_HEIGHT_LIMIT 250.0
#define DEFAULT_RETRIES_LIMIT 60

#include <algorithm>
#include "Identification_Method.h"
#include "../SensorsModule.h"


class Height_Method : public Identification_Method {
public:
    void ClearTemplate();
    void BeginSaveTemplate();
    void ContinueSaveTemplate();
    void Update();
    double RateUser(XnUserID userId);
    void LateUpdate();

private:
    std::vector<std::list<double>> userHeightSamples;
    int numberOfCollectedsamples = 0;
    int retries = 0;
    double originalHeight = 0.0;
    double CalculateHeight(XnUserID const& userId);
    double CalculateHeight(XnUserID const& userId, double &confidence);
    double CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB, double &confidence);
};

#endif //ELEKTRON_ESCORT_HEIGHT_METHOD_H
