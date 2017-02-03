#ifndef ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H
#define ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H

#include <XnCppWrapper.h>
#include "../../Common.h"
#include "../DataStorage.h"


class IdentificationMethod {
public:
    float trustValue = 0.0f;
    virtual void ClearTemplate()=0;
    virtual bool SaveTemplate()=0;
    virtual void Update()=0;
    virtual float RateUser(XnUserID userId)=0;
    virtual void LateUpdate()=0;
};

#endif //ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H
