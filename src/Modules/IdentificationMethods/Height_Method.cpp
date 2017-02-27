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
    //DEBUG START
    timer=0;
    counter=0;
    templateCreationDistance=0.0;
    //DEBUG END
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
            //DEBUG START
            XnPoint3D position;
            SensorsModule::GetInstance().GetUserGenerator().GetCoM(DataStorage::GetInstance().GetCurrentUserXnId(), position);
            templateCreationDistance+=position.Z;
            //DEBUG END
        }
        else {
            ++retries;
            if(retries >= DEFAULT_RETRIES_LIMIT) {
                state = NotReady;
            }
        }
    }
    else if(numberOfCollectedsamples >= DEFAULT_NUMBER_OF_TEMPLATE_SAMPLES) {
        originalHeight = originalHeight/numberOfCollectedsamples;
        state = Ready;
        //DEBUG START
        templateCreationDistance=templateCreationDistance/numberOfCollectedsamples;
        ROS_ERROR("Calculated template: %f; At distance: %f", originalHeight, templateCreationDistance);
        //DEBUG END
    }
}

void Height_Method::Update() {
}

double Height_Method::RateUser(XnUserID userId) {
    double confidence;
    double userHeight = CalculateHeight(userId, confidence);
    double result = userHeight - originalHeight;
    //DEBUG START
    if(timer <= 0 && counter < 600) {
        timer = 0;
        XnPoint3D position;
        SensorsModule::GetInstance().GetUserGenerator().GetCoM(DataStorage::GetInstance().GetCurrentUserXnId(), position);
        ROS_ERROR("diff_walk_obr = [diff_walk_obr; %f, %f];", position.Z, result);
        counter++;
    }
    if(timer > 0) {
        --timer;
    }
    //DEBUG END
    if(abs(userHeight - originalHeight) <= DEFAULT_HEIGHT_TOLERANCE) {
        return 1.0;
    }
    else {
        return 0.0;
    }
}

void Height_Method::LateUpdate() {
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Private
///////////////////////////////////////////////////////////////////////////////////////////////////////////
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

double Height_Method::CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB, double &confidence)
{
    XnSkeletonJointPosition joint_A_Postition;
    XnSkeletonJointPosition joint_B_Postition;
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().GetSkeletonJointPosition(userId, jointA, joint_A_Postition);
    SensorsModule::GetInstance().GetUserGenerator().GetSkeletonCap().GetSkeletonJointPosition(userId, jointB, joint_B_Postition);
    double xDistance = abs(joint_A_Postition.position.X - joint_B_Postition.position.X);
    double yDistance = abs(joint_A_Postition.position.Y - joint_B_Postition.position.Y);
    double zDistance = abs(joint_A_Postition.position.Z - joint_B_Postition.position.Z);
    confidence = std::min(joint_A_Postition.fConfidence, joint_B_Postition.fConfidence);
    return sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance));
}