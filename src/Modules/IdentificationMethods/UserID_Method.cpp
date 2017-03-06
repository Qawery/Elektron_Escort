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
        repeats = REPEATS_LIMIT;
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
    if(originalId != NO_USER) {
        if(originalId == userId) {
            return (repeats/REPEATS_LIMIT);
        }
    }
    return 0.0;
}

void UserID_Method::LateUpdate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        if(originalId == DataStorage::GetInstance().GetCurrentUserXnId())
        {
            if(repeats < REPEATS_LIMIT) {
                repeats++;
            }
        }
        else {
            originalId = DataStorage::GetInstance().GetCurrentUserXnId();
            repeats = 0;
        }

    }
}