#ifndef ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H
#define ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H

#include <XnCppWrapper.h>
#include "../../Common.h"
#include "../DataStorage.h"


class Identification_Method {
public:
    double trustValue = 0.0;
    virtual void ClearTemplate()=0;
    virtual bool SaveTemplate()=0;
    virtual void Update()=0;
    virtual double RateUser(XnUserID userId)=0;
    virtual void LateUpdate()=0;
};

#endif //ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H
