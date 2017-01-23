#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define NO_USER -1
#define MAX_USERS 15
#define POSE_COOLDOWN_TIME 3
#define MAIN_LOOP_RATE 30

using std::string;


/**
Spis treści:
-Definicje, zmienne i obiekty globalne
-XN Callbacks
-Inicjalizacja, finalizacja i główna funkcja programu
*/


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Definicje, zmienne i obiekty globalne///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


ros::NodeHandle* nodeHandlePublic;	//Node handle z nazwą w przestrzeni publicznej
ros::NodeHandle* nodeHandlePrivate;	//Node handle z nazwą w przestrzeni prywatnej

xn::Context context;
xn::DepthGenerator depthGenerator;
xn::UserGenerator userGenerator;

XnCallbackHandle userCallbacksHandle;
XnCallbackHandle calibrationCallbacksHandle;
XnCallbackHandle poseCallbacksHandle;
XnBool isCalibrationPoseNeeded = FALSE;
XnChar startingPose[20] = "";

string parentFrame("escort_main");	//TODO podpięcie pod kinecta
bool isCalibrationFilePresent = false;
float poseCooldownForUsers[MAX_USERS];


//DEBUG
XnUserID currentUserId = NO_USER;

#define DEBUG_TIME 0.5
#define MIN_DISTANCE 1000.0
#define MAX_DISTANCE 4000.0
#define DISTANCE_STEP 1000.0
#define MAX_MEASURMENTS 100.0

class DistanceMeasurments
{
public:
	double no;
	double min;
	double avg;
	double max;

	DistanceMeasurments();
};

DistanceMeasurments::DistanceMeasurments()
{
	no = 0;
	min = 0;
	avg = 0;
	max = 0;
}

float debugTimeCooldown = 0.0f;
std::vector<DistanceMeasurments> measurments;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////XN Callbacks////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    ROS_INFO("New User: %d", userId);
    generator.GetPoseDetectionCap().StartPoseDetection("Psi", userId);

    if(isCalibrationFilePresent)
    {
        ROS_INFO("Loaded calibration data from file.");
        generator.GetSkeletonCap().LoadCalibrationDataFromFile(userId, "calibration.bin");
        generator.GetSkeletonCap().StartTracking(userId);
    }
}


void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID userId, void* cookie)
{
    ROS_INFO("Lost User: %d", userId);
    if(currentUserId == userId)
    {
        ROS_INFO("Lost current user: %d", currentUserId);
        currentUserId = NO_USER;
        if(isCalibrationFilePresent)
        {
            ROS_INFO("Removed calibration file.");
            isCalibrationFilePresent = false;
            remove("calibration.bin");
        }
    }
}


void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID userId, void* pCookie)
{
    if(userId < MAX_USERS && poseCooldownForUsers[userId] <= 0)
    {
        ROS_INFO("Pose detected for user: %d", userId);
        poseCooldownForUsers[userId] = POSE_COOLDOWN_TIME;

        if(currentUserId == NO_USER)
        {
            ROS_INFO("Requested calibration for possible current user.");
            userGenerator.GetPoseDetectionCap().StopPoseDetection(userId);
            userGenerator.GetSkeletonCap().RequestCalibration(userId, TRUE);
        }
        else if(currentUserId == userId)
        {
            ROS_INFO("Unregistered current user: %d", currentUserId);
            currentUserId = NO_USER;
            if(isCalibrationFilePresent)
            {
                ROS_INFO("Removed calibration file.");
                isCalibrationFilePresent = false;
                remove("calibration.bin");
            }
        }
    }
    else if(userId >= MAX_USERS)
    {
        ROS_INFO("Pose detected for user: %d; But ignored due to user limit.", userId);
    }
    else
    {
        ROS_INFO("Pose detected for user: %d; But ignored due to cooldown.", userId);
    }
}


void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID userId, void* cookie)
{
    ROS_INFO("Calibration start for user: %d", userId);
}


void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID userId, XnBool success, void* cookie)
{
    ROS_INFO("Calibration end for user: %d", userId);
    if (success)
    {
        ROS_INFO("Successful calibration, start tracking user: %d", userId);
        userGenerator.GetSkeletonCap().StartTracking(userId);
        if(currentUserId == NO_USER)
        {
            currentUserId = userId;
            ROS_INFO("New current user: %d", currentUserId);
            userGenerator.GetSkeletonCap().SaveCalibrationDataToFile(userId, "calibration.bin");
            isCalibrationFilePresent = true;
        }
    }
    else
    {
        ROS_INFO("Failed calibration for user: %d", userId);
    }
    userGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", userId);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////                                                    ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double CalculateJointDistance(XnUserID const& userId, XnSkeletonJoint const& jointA, XnSkeletonJoint const& jointB)
{
    if(userId > 0)
    {
        XnSkeletonJointPosition joint_A_Postition;
        XnSkeletonJointPosition joint_B_Postition;
        userGenerator.GetSkeletonCap().GetSkeletonJointPosition(userId, jointA, joint_A_Postition);
        userGenerator.GetSkeletonCap().GetSkeletonJointPosition(userId, jointB, joint_B_Postition);
        double xDistance = 0.0;
        double yDistance = 0.0;
        double zDistance = 0.0;
        xDistance = abs(joint_A_Postition.position.X - joint_B_Postition.position.X);
        yDistance = abs(joint_A_Postition.position.Y - joint_B_Postition.position.Y);
        zDistance = abs(joint_A_Postition.position.Z - joint_B_Postition.position.Z);

        return sqrt((xDistance*xDistance)+(yDistance*yDistance)+(zDistance*zDistance));
    }
    return 0.0;
}

//Debug
void DisplayResults()
{
	int distance = 0;
	for(int i=0; i < measurments.size() ; i++)
	{
		distance = (i * DISTANCE_STEP) + MIN_DISTANCE;
		ROS_INFO("Distance %d no: %f; min: %f; ang: %f; max: %f", distance, measurments[i].no, measurments[i].min, measurments[i].avg, measurments[i].max);
	}
}

//Debug
void DebugInformation()
{
	bool finish = true;
	for(int i=0; i < measurments.size() ; i++)
	{
		if(measurments[i].no < MAX_MEASURMENTS)
		{
			finish = false;
			break;
		}
	}

	if(debugTimeCooldown > 0.0f)
	{
		debugTimeCooldown = debugTimeCooldown - (1.0f/MAIN_LOOP_RATE);
	}
	if(debugTimeCooldown <= 0.0f)
	{
		debugTimeCooldown = DEBUG_TIME;

		XnPoint3D userCOM;
		userGenerator.GetCoM(currentUserId, userCOM);
		ROS_INFO("Current distance: %f", userCOM.Z);

		if(finish)
		{
			DisplayResults();
		}
		else
		{
			int distance = 0;
			for(int i=0; i < measurments.size() ; i++)
			{
				distance = (i * DISTANCE_STEP) + MIN_DISTANCE;
				ROS_INFO("Distance %d number of measurments: %f", distance, measurments[i].no);
			}
		}
	}

	if(!finish && userGenerator.GetSkeletonCap().IsTracking(currentUserId))
	{
		XnPoint3D currentUserCOM;
		userGenerator.GetCoM(currentUserId, currentUserCOM);

		if(currentUserCOM.Z >= MIN_DISTANCE && currentUserCOM.Z < MAX_DISTANCE)
		{
			int index = floor((currentUserCOM.Z - MIN_DISTANCE)/DISTANCE_STEP);

			if(measurments[index].no < MAX_MEASURMENTS)
			{
				double currentMeasurment = CalculateJointDistance(currentUserId, XN_SKEL_HEAD, XN_SKEL_TORSO);
				double newAverage = ((measurments[index].no * measurments[index].avg) + currentMeasurment) / (measurments[index].no + 1);
				measurments[index].avg = newAverage;
				measurments[index].no++;
				if(measurments[index].min > currentMeasurment)
				{
					measurments[index].min = currentMeasurment;
				}
				if(measurments[index].max < currentMeasurment)
				{
					measurments[index].max = currentMeasurment;
				}
			}
		}
	}
}

void Update()
{
	//Pose detection cooldown.
	for(int i=0; i < MAX_USERS; i++)
	{
		if(poseCooldownForUsers[i] > 0.0f)
		{
			poseCooldownForUsers[i] = poseCooldownForUsers[i] - (1.0f/MAIN_LOOP_RATE);
		}
	}

	//Debug
	DebugInformation();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////TF/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id)
{
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    userGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
                           m[3], m[4], m[5],
                           m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}


void publishTransforms(const std::string& frame_id)
{
	XnUserID users[MAX_USERS];
	XnUInt16 users_count = MAX_USERS;
	userGenerator.GetUsers(users, users_count);

	for (int i = 0; i < users_count; ++i)
	{
		XnUserID user = users[i];
		if (userGenerator.GetSkeletonCap().IsTracking(user))
		{
			publishTransform(user, XN_SKEL_HEAD,		   frame_id, "head");
			publishTransform(user, XN_SKEL_NECK,		   frame_id, "neck");
			publishTransform(user, XN_SKEL_TORSO,		  frame_id, "torso");

			publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
			publishTransform(user, XN_SKEL_LEFT_ELBOW,	 frame_id, "left_elbow");
			publishTransform(user, XN_SKEL_LEFT_HAND,	  frame_id, "left_hand");

			publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
			publishTransform(user, XN_SKEL_RIGHT_ELBOW,	frame_id, "right_elbow");
			publishTransform(user, XN_SKEL_RIGHT_HAND,	 frame_id, "right_hand");

			publishTransform(user, XN_SKEL_LEFT_HIP,	   frame_id, "left_hip");
			publishTransform(user, XN_SKEL_LEFT_KNEE,	  frame_id, "left_knee");
			publishTransform(user, XN_SKEL_LEFT_FOOT,	  frame_id, "left_foot");

			publishTransform(user, XN_SKEL_RIGHT_HIP,	  frame_id, "right_hip");
			publishTransform(user, XN_SKEL_RIGHT_KNEE,	 frame_id, "right_knee");
			publishTransform(user, XN_SKEL_RIGHT_FOOT,	 frame_id, "right_foot");
		}
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Inicjalizacja, finalizacja i główna funkcja programu////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
	Inicializuje parametry i obiekty wykorzystywane w programie.
	Zwraca "true" w przypadku powodzenia, a "false" w przypadku porażki.
*/
bool Initialization()
{
	for(int i=0; i < MAX_USERS; i++)
	{
		poseCooldownForUsers[i] = 0.0f;
	}

	nodeHandlePublic = new ros::NodeHandle();	//Utworzenie node handle z nazwą publiczną.
	nodeHandlePrivate = new ros::NodeHandle("~");	//Utworzenie node handle z nazwą prywatną.

	string configFilename = ros::package::getPath("elektron_escort") + "/OpenNi_config.xml";	//Pobranie nazwy pliku konfiguracyjnego.

	XnStatus result = context.InitFromXmlFile(configFilename.c_str());	//Initializacja z pliku konfiguracyjnego.
	if (result != XN_STATUS_OK)
	{
		ROS_ERROR("Initialization from Xml file failed: %s", xnGetStatusString(result));
		return false;
	}

	result = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);	//Próba znalezienia generatora obrazu głębi.
	if (result != XN_STATUS_OK)
	{
		ROS_ERROR("Find depth generator failed: %s", xnGetStatusString(result));
		return false;
	}

	result = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);	//Próba znalezienia genaratora użytkowników.
	if (result != XN_STATUS_OK)
	{
		result = userGenerator.Create(context);	//Nie znaleziono istniejącego generatora użytkowników, próbujemy utworzyć własny.
	}
	if (result != XN_STATUS_OK)
	{
		ROS_ERROR("Create user generator failed: %s", xnGetStatusString(result));
		return false;
	}

	if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		ROS_INFO("User generator doesn't support skeleton");
		return false;
	}

	userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, userCallbacksHandle);

	userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, calibrationCallbacksHandle);

	if (userGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		isCalibrationPoseNeeded = TRUE;
		if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			ROS_INFO("Calibration pose required, but not supported");
			return false;
		}

		userGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, poseCallbacksHandle);
		userGenerator.GetSkeletonCap().GetCalibrationPose(startingPose);
	}

	userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	result = context.StartGeneratingAll();
	if (result != XN_STATUS_OK)
	{
		ROS_ERROR("Start generating all failed: %s", xnGetStatusString(result));
		return false;
	}

	nodeHandlePrivate->getParam("camera_frame_id", parentFrame);

	return true;
}


/**
	Zwalnia używane wskaźniki.
*/
void Finish()
{
	context.Release();
	if(isCalibrationFilePresent)
	{
		remove("calibration.bin");
	}

	delete nodeHandlePublic;
	delete nodeHandlePrivate;
}


/**
	Główna funkcja programu.
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "escort_main");	//Inicializacja ROS.

	//Debug
	measurments.reserve(ceil((MAX_DISTANCE-MIN_DISTANCE)/DISTANCE_STEP));
	measurments.resize(ceil((MAX_DISTANCE-MIN_DISTANCE)/DISTANCE_STEP));

	if(Initialization())
	{
		ros::Rate mainLoopRate(MAIN_LOOP_RATE);	//TODO: pobranie loop rate a launchfile
		while (ros::ok())
		{
			context.WaitAndUpdateAll();
			publishTransforms(parentFrame);
			Update();
			mainLoopRate.sleep();
		}
	}
	Finish();

	return 0;
}
