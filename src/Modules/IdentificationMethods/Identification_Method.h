#ifndef ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H
#define ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H

#include <XnCppWrapper.h>
#include "../../Common.h"
#include "../DataStorage.h"


enum MethodState {
    NotReady, CreatingTemplate, Ready
};

class Identification_Method {
public:
    double trustValue = 0.0;
    virtual void ClearTemplate()=0;
    virtual void BeginSaveTemplate()=0;
    virtual void ContinueSaveTemplate()=0;
    virtual void Update()=0;
    virtual double RateUser(XnUserID userId)=0;
    virtual void LateUpdate()=0;
    MethodState GetState();

protected:
    MethodState state = NotReady;
};

#endif //ELEKTRON_ESCORT_IDENTIFICATIONMETHOD_H
