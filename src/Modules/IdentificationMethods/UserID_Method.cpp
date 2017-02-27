#include "UserID_Method.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void UserID_Method::ClearTemplate() {
    originalId = NO_USER;
}

void UserID_Method::BeginSaveTemplate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        originalId = DataStorage::GetInstance().GetCurrentUserXnId();
        state = Ready;
    }
    else {
        state = NotReady;
    }

}

void UserID_Method::ContinueSaveTemplate() {
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