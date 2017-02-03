#include "UserID_Method.h"

void UserID_Method::ClearTemplate() {
    previousId = NO_USER;
}

bool UserID_Method::SaveTemplate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        previousId = DataStorage::GetInstance().GetCurrentUserXnId();
        return true;
    }
    else {
        return false;
    }

}

void UserID_Method::Update() {

}

float UserID_Method::RateUser(XnUserID userId) {
    if(previousId == userId) {
        return 1.0f;
    }
    else {
        return 0.0f;
    }
}

void UserID_Method::LateUpdate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        previousId = DataStorage::GetInstance().GetCurrentUserXnId();
    }
}