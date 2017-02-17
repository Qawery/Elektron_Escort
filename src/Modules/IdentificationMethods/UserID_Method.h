#ifndef ELEKTRON_ESCORT_USERID_METHOD_H
#define ELEKTRON_ESCORT_USERID_METHOD_H

#include "Identification_Method.h"


class UserID_Method: public Identification_Method {
public:
    void ClearTemplate();
    void BeginSaveTemplate();
    void ContinueSaveTemplate();
    void Update();
    double RateUser(XnUserID userId);
    void LateUpdate();

private:
    XnUserID originalId;
};

#endif //ELEKTRON_ESCORT_USERID_METHOD_H
