#include "UserID_Method.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void UserID_Method::ClearTemplate() {
    originalId = NO_USER;
}

bool UserID_Method::SaveTemplate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        originalId = DataStorage::GetInstance().GetCurrentUserXnId();
        return true;
    }
    else {
        return false;
    }

}

void UserID_Method::Update() {

}

double UserID_Method::RateUser(XnUserID userId) {
    if(originalId != NO_USER && originalId == userId) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

void UserID_Method::LateUpdate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        originalId = DataStorage::GetInstance().GetCurrentUserXnId();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////