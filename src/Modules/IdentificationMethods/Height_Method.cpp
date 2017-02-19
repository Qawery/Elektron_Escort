#include "Height_Method.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Public
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void Height_Method::ClearTemplate() {
    originalHeight = 0.0;
}

void Height_Method::BeginSaveTemplate() {
    numberOfCollectedsamples = 0;
    originalHeight = 0.0;
    state = CreatingTemplate;
    retries = 0;
}

void Height_Method::ContinueSaveTemplate() {
    if(state != CreatingTemplate) {
        return;
    }
    if(numberOfCollectedsamples < DEFAULT_NUMBER_OF_TEMPLATE_SAMPLES) {
        double confidence;
        double heightSample = CalculateHeight(DataStorage::GetInstance().GetCurrentUserXnId(), confidence);
        if(confidence >= 1.0) {
            originalHeight += heightSample;
            ++numberOfCollectedsamples;
            retries = 0;
            //DEBUG
            //ROS_INFO("Sample H/C: %f/%f", heightSample, confidence);
        }
        else {
            ++retries;
            if(retries >= DEFAULT_RETRIES_LIMIT) {
                state = NotReady;
            }
        }
    }
    else if(numberOfCollectedsamples == DEFAULT_NUMBER_OF_TEMPLATE_SAMPLES) {
        originalHeight = originalHeight/numberOfCollectedsamples;
        state = Ready;
    }
}

void Height_Method::Update() {
}

double Height_Method::RateUser(XnUserID userId) {
    double confidence;
    double userHeight = CalculateHeight(userId, confidence);
    //DEBUG
    //ROS_INFO("User I/H/C: %d/%f/%f", userId, userHeight, confidence);
    if(abs(userHeight - originalHeight) <= DEFAULT_HEIGHT_TOLERANCE && confidence >= 1.0) {
        return 1.0;
    }
    else if(abs(userHeight - originalHeight) <= 2*DEFAULT_HEIGHT_TOLERANCE) {
        return 0.5;
    }
    else {
        return 0.0;
    }

}

void Height_Method::LateUpdate() {
}

double Height_Method::CalculateHeight(XnUserID const& userId) {
    double confidence;
    return CalculateHeight(userId, confidence);
}

double Height_Method::CalculateHeight(XnUserID const& userId, double &confidence) {
    double result = 0.0;
    double confidenceTemp;
    double leftSide = 0.0;
    double leftSideConfidence;
    leftSide += CalculateJointDistance(userId, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP, confidenceTemp);
    leftSideConfidence = confidenceTemp;
    leftSide += CalculateJointDistance(userId, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE, confidenceTemp);
    leftSideConfidence += confidenceTemp;
    leftSide += CalculateJointDistance(userId, XN_SKEL_LEFT_KNEE,XN_SKEL_LEFT_FOOT, confidenceTemp);
    leftSideConfidence += confidenceTemp;
    double rightSide = 0.0;
    double rightSideConfidence;
    rightSide += CalculateJointDistance(userId, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP, confidenceTemp);
    rightSideConfidence = confidenceTemp;
    rightSide += CalculateJointDistance(userId, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE, confidenceTemp);
    rightSideConfidence += confidenceTemp;
    rightSide += CalculateJointDistance(userId, XN_SKEL_RIGHT_KNEE,XN_SKEL_RIGHT_FOOT, confidenceTemp);
    rightSideConfidence += confidenceTemp;
    if(rightSideConfidence > leftSideConfidence) {
        result += rightSide;
        confidence = leftSideConfidence;
    }
    else if(leftSideConfidence > rightSideConfidence) {
        result += leftSide;
        confidence = rightSideConfidence;
    }
    else {
        result += (leftSide+rightSide)/2;
        confidence = rightSideConfidence;
    }
    result += CalculateJointDistance(userId, XN_SKEL_HEAD, XN_SKEL_NECK, confidenceTemp);
    confidence += confidenceTemp;
    result += CalculateJointDistance(userId, XN_SKEL_NECK, XN_SKEL_TORSO, confidenceTemp);
    confidence += confidenceTemp;
    confidence = confidence/5;
    return result;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
double Height_Method::CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB, double &confidence)
{
    XnSkeletonJointPosition joint_A_Postition;
    XnSkeletonJointPosition joint_B_Postition;
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().GetSkeletonJointPosition(userId, jointA, joint_A_Postition);
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().GetSkeletonJointPosition(userId, jointB, joint_B_Postition);
    double xDistance = 0.0;
    double yDistance = 0.0;
    double zDistance = 0.0;
    xDistance = abs(joint_A_Postition.position.X - joint_B_Postition.position.X);
    yDistance = abs(joint_A_Postition.position.Y - joint_B_Postition.position.Y);
    zDistance = abs(joint_A_Postition.position.Z - joint_B_Postition.position.Z);
    confidence = std::min(joint_A_Postition.fConfidence, joint_B_Postition.fConfidence);
    return sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance));
}