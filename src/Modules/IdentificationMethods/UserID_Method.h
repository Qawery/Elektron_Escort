#ifndef ELEKTRON_ESCORT_USERID_METHOD_H
#define ELEKTRON_ESCORT_USERID_METHOD_H

#include "IdentificationMethod.h"


class UserID_Method : public IdentificationMethod {
public:
    void ClearTemplate();
    bool SaveTemplate();
    void Update();
    double RateUser(XnUserID userId);
    void LateUpdate();

private:
    XnUserID previousId;
};

#endif //ELEKTRON_ESCORT_USERID_METHOD_H
