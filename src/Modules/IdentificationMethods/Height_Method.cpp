#include "Height_Method.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void Height_Method::ClearTemplate() {
    originalHeight = 0.0;
}

bool Height_Method::SaveTemplate() {
    if(DataStorage::GetInstance().GetCurrentUserXnId() != NO_USER) {
        double confidence = 0.0;
        originalHeight = CalculateHeight(DataStorage::GetInstance().GetCurrentUserXnId(), confidence);
        if(confidence >= 1.0) {
            return true;
        }
        else{
            originalHeight = 0.0;
            return false;
        }
    }
    else {
        return false;
    }

}

void Height_Method::Update() {
}

double Height_Method::RateUser(XnUserID userId) {
    double heightConfidence;
    double userHeight = CalculateHeight(userId, heightConfidence);
    if(abs(userHeight - originalHeight) <= DEFAULT_HEIGHT_TOLERANCE && heightConfidence >= 1.0) {
        return 1.0;
    }
    else if(abs(userHeight - originalHeight) <= DEFAULT_HEIGHT_TOLERANCE && heightConfidence >= 0.5) {
        return 0.5;
    }
    else {
        return 0.0;
    }

}

void Height_Method::LateUpdate() {
}

double Height_Method::CalculateHeight(XnUserID const& userId) {
    double confidence = 0.0;
    return CalculateHeight(userId, confidence);
}

double Height_Method::CalculateHeight(XnUserID const& userId, double& confidence) {
    double result = 0.0;
    double confidenceTemp = 1.0;
    double leftSide = 0.0;
    double leftSideConfidence = 1.0;
    leftSide += CalculateJointDistance(userId, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP, confidenceTemp);
    leftSideConfidence = std::min(leftSideConfidence, confidenceTemp);
    leftSide += CalculateJointDistance(userId, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE, confidenceTemp);
    leftSideConfidence = std::min(leftSideConfidence, confidenceTemp);
    leftSide += CalculateJointDistance(userId, XN_SKEL_LEFT_KNEE,XN_SKEL_LEFT_FOOT, confidenceTemp);
    leftSideConfidence = std::min(leftSideConfidence, confidenceTemp);
    double rightSide = 0.0;
    double rightSideConfidence = 1.0;
    rightSide += CalculateJointDistance(userId, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP, confidenceTemp);
    rightSideConfidence = std::min(rightSideConfidence, confidenceTemp);
    rightSide += CalculateJointDistance(userId, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE, confidenceTemp);
    rightSideConfidence = std::min(rightSideConfidence, confidenceTemp);
    rightSide += CalculateJointDistance(userId, XN_SKEL_RIGHT_KNEE,XN_SKEL_RIGHT_FOOT, confidenceTemp);
    rightSideConfidence = std::min(rightSideConfidence, confidenceTemp);
    if(leftSideConfidence > rightSideConfidence) {
        confidence = leftSideConfidence;
        result += leftSide;
    }
    else if (rightSideConfidence > leftSideConfidence) {
        confidence = rightSideConfidence;
        result += rightSide;
    }
    else {
        confidence = rightSideConfidence;
        result += (leftSide+rightSide)/2;
    }
    result += CalculateJointDistance(userId, XN_SKEL_HEAD, XN_SKEL_NECK, confidenceTemp);
    confidence = std::min(confidence, confidenceTemp);
    result += CalculateJointDistance(userId, XN_SKEL_NECK, XN_SKEL_TORSO, confidenceTemp);
    confidence = std::min(confidence, confidenceTemp);
    return result;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
double Height_Method::CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB, double& confidence)
{
    XnSkeletonJointPosition joint_A_Postition;
    XnSkeletonJointPosition joint_B_Postition;
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().GetSkeletonJointPosition(userId, jointA, joint_A_Postition);
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().GetSkeletonJointPosition(userId, jointB, joint_B_Postition);
    confidence = std::min(confidence = joint_A_Postition.fConfidence, confidence = joint_B_Postition.fConfidence);
    double xDistance = 0.0;
    double yDistance = 0.0;
    double zDistance = 0.0;
    xDistance = abs(joint_A_Postition.position.X - joint_B_Postition.position.X);
    yDistance = abs(joint_A_Postition.position.Y - joint_B_Postition.position.Y);
    zDistance = abs(joint_A_Postition.position.Z - joint_B_Postition.position.Z);
    return sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance));
}