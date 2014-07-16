#include <ros/ros.h>
#include <convert_image/ConvertImage.hpp>

#include <opencv2/opencv.hpp>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

#include <std_msgs/Float64MultiArray.h>

cv::Mat img;
bool img_available = false;

void selectedCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	img_available = true;
	ConvertImage::MsgToMatCopy(msg, img);
}

float yaw, pitch;
bool angles_available = false;
void setHeadAnglesCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	angles_available = true;
	yaw = msg->data[0];
	pitch = msg->data[1];
}

int main(int argc, char** argv)
{
	//***************************************************
	//*************** Initialise ROS ********************
	//***************************************************
	ros::init(argc, argv, "edinferno_coach_calibration");
	ros::NodeHandle n("~");
	ros::Publisher angles_pub = n.advertise<std_msgs::Float64MultiArray>("/ed_coach/get_head_angles", 2);
	ros::Subscriber angles_sub = n.subscribe("/ed_coach/set_head_angles", 1, setHeadAnglesCallback);

	ros::Rate r(10);


	//***************************************************
	//******** Initialise a NaoQi motion proxy **********
	//***************************************************

	const AL::ALValue joint_names = AL::ALValue::array("HeadYaw", "HeadPitch");

    // Create an ALMotionProxy to call the methods to move NAO's head.
    AL::ALMotionProxy motion("127.0.0.1", 9559);


    // Ensure sitting position
    // motion.wakeUp();
    motion.setStiffnesses(AL::ALValue::array("LHipYawPitch", "LHipRoll", "LHipPitch",
                          					 "RHipYawPitch", "RHipRoll", "RHipPitch"),
    					  AL::ALValue::array(0.3f, 0.3f, 0.3f,
    					                     0.3f, 0.3f, 0.3f));

    // Make sure the head is stiff to be able to move it
    motion.setStiffnesses(joint_names, AL::ALValue::array(1.0f, 1.0f));

    // Make sure the head is in initial position
    float head_yaw = 0.0f, head_pitch = 0.0f;
    float d_yaw, d_pitch;
	motion.setAngles(joint_names, AL::ALValue::array(head_yaw, head_pitch), 0.1f);
	const float HEAD_YAW_MIN = -1.5f, HEAD_YAW_MAX = 1.5f, HEAD_PITCH_MIN = -0.45f, HEAD_PITCH_MAX = 0.45f;

	std_msgs::Float64MultiArray msg;

	while(ros::ok())
	{
		std::vector<float> angles = motion.getAngles(joint_names, true);
		msg.data.clear();
		msg.data.push_back(angles[0]);
		msg.data.push_back(angles[1]);

		angles_pub.publish(msg);

		if(angles_available)
		{
			motion.setAngles(joint_names, AL::ALValue::array(yaw, pitch), 0.1f);
			angles_available = false;
		}

		ros::spinOnce();
		r.sleep();
	}
}

		// 		// if(ball_not_seen >= REMEMBER_BALL)
		// 	{
		// 		std::vector<float> angls = motion.getAngles(joint_names, true);

		// 		float err_yaw = fabs(angls[0] - searching_head_yaw);
		// 		float err_pitch = fabs(angls[1] - searching_head_pitch);

		// 		// Check if new angles should be picked
		// 		if(ball_not_seen == REMEMBER_BALL ||
		// 		   (err_yaw < 0.25 && err_pitch < 0.25))
		// 		{

		// 			searching_head_yaw = HEAD_YAW_MIN + (HEAD_YAW_MAX - HEAD_YAW_MIN) * ((rand() % 101) / 100.0f);
		// 			searching_head_pitch = HEAD_PITCH_MIN + (HEAD_PITCH_MAX - HEAD_PITCH_MIN) * ((rand() % 101) / 100.0f);

		// 			motion.setAngles(joint_names, AL::ALValue::array(searching_head_yaw, searching_head_pitch), 0.1f);
		// 		}
		// 	}

  //   		ConvertImage::MatToMsgCopy(img, "/world", msg);
  //   		seg_pub.publish(msg);

  //   		// std::cout << "[edinferno_coach.cpp] Published " << std::endl;
		// }



		// else
		// {
		// 	// std::cout << "[edinferno_coach.cpp] No image " << std::endl;
		// }


// 		// std::cout << "[edinferno_coach.cpp] SpinOnce " << std::endl;
// 		ros::spinOnce();
// 		// std::cout << "[edinferno_coach.cpp] Sleep " << std::endl;
// 		r.sleep();
// 	}
// 	motion.setStiffnesses(joint_names, AL::ALValue::array(0.0f, 0.0f));

// 	return 0;
// }
