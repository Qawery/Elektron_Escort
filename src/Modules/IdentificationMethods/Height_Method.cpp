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
}

void Height_Method::ContinueSaveTemplate() {
    if(state != CreatingTemplate) {
        return;
    }
    if(numberOfCollectedsamples < DEFAULT_NUMBER_OF_TEMPLATE_SAMPLES) {
        //DEBUG
        //originalHeight += CalculateHeight(DataStorage::GetInstance().GetCurrentUserXnId());
        double temp = CalculateHeight(DataStorage::GetInstance().GetCurrentUserXnId());
        originalHeight += temp;
        ROS_INFO("Temp: %f", temp);
        ++numberOfCollectedsamples;
    }
    else if(numberOfCollectedsamples == DEFAULT_NUMBER_OF_TEMPLATE_SAMPLES) {
        originalHeight = originalHeight/numberOfCollectedsamples;
        //DEBUG
        ROS_INFO("Final: %f", originalHeight);
        state = Ready;
    }
}

void Height_Method::Update() {
}

double Height_Method::RateUser(XnUserID userId) {
    double userHeight = CalculateHeight(userId);
    if(abs(userHeight - originalHeight) <= DEFAULT_HEIGHT_TOLERANCE) {
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
    double result = 0.0;
    double leftSide = 0.0;
    leftSide += CalculateJointDistance(userId, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
    leftSide += CalculateJointDistance(userId, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
    leftSide += CalculateJointDistance(userId, XN_SKEL_LEFT_KNEE,XN_SKEL_LEFT_FOOT);
    double rightSide = 0.0;
    rightSide += CalculateJointDistance(userId, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
    rightSide += CalculateJointDistance(userId, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
    rightSide += CalculateJointDistance(userId, XN_SKEL_RIGHT_KNEE,XN_SKEL_RIGHT_FOOT);
    result += (leftSide+rightSide)/2;
    result += CalculateJointDistance(userId, XN_SKEL_HEAD, XN_SKEL_NECK);
    result += CalculateJointDistance(userId, XN_SKEL_NECK, XN_SKEL_TORSO);
    return result;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
double Height_Method::CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB)
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
    return sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance));
}