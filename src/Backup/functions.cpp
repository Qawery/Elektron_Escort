void DebugInformation()
{
	//Print current user number.
	if(printCurrentUserNumberCooldown > 0.0f)
	{
		printCurrentUserNumberCooldown = printCurrentUserNumberCooldown - (1.0f/MAIN_LOOP_RATE);
	}
	if(printCurrentUserNumberCooldown <= 0.0f)
	{
		printCurrentUserNumberCooldown = CURRENT_USER_NUMBER_PRINT_TIME;

		XnUserID users[MAX_USERS];
		XnUInt16 users_count = MAX_USERS;

		userGenerator.GetUsers(users, users_count);

		for (int i = 0; i < users_count; ++i) 
		{
			if(userGenerator.GetSkeletonCap().IsTracking(users[i]))
			{
				ROS_INFO("Is tracking");
			}
		}
	}

	/*
	//Print current user parameters.
	if(printCurrentUserParametersCooldown > 0.0f)
	{
		printCurrentUserParametersCooldown = printCurrentUserParametersCooldown - (1.0f/MAIN_LOOP_RATE);
	}
	if(printCurrentUserParametersCooldown <= 0.0f)
	{
		printCurrentUserParametersCooldown = CURRENT_USER_PARAMETERS_PRINT_TIME;

		XnPoint3D currentUserCOM;
		userGenerator.GetCoM(currentUserId, currentUserCOM);

		ROS_INFO("Current user: %d. Center of mass position: X %f; Y %f; Z %f;", currentUserId, currentUserCOM.X, currentUserCOM.Y, currentUserCOM.Z);

		ROS_INFO("Head to Neck distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_HEAD, XN_SKEL_NECK));
		ROS_INFO("Neck to Torso distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_NECK, XN_SKEL_TORSO));
		
		ROS_INFO("Torso to Left Hip distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP));
		ROS_INFO("Left Hip to Left Knee distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE));
		ROS_INFO("Left Knee to Left Foot distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT));

		ROS_INFO("Torso to Right Hip distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP));
		ROS_INFO("Right Hip to Right Knee distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE));
		ROS_INFO("Right Knee to Right Foot distance: %f", CalculateJointDistance(currentUserId, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT));
	}
	*/	
}
