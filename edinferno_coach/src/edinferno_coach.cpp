#include <ros/ros.h>
#include <convert_image/ConvertImage.hpp>

#include <opencv2/opencv.hpp>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

#include "segmentation.hpp"
#include "ball_detection.hpp"
#include "lsd_opencv.hpp"


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "SPLCoachMessage.hpp"



// LSD Example
// ros::Publisher lines_pub = n.advertise<sensor_msgs::Image>("lines", 2);
	// cv::Ptr<cv::LineSegmentDetector> lsd_std =
		// cv::createLineSegmentDetectorPtr(cv::LSD_REFINE_NONE);

	// while
	//
	// ConvertImage::MatToMsgCopy(img, "/world", msg);
			// lines_pub.publish(msg);

			// cv::cvtColor(img, img, CV_RGB2GRAY);
			// std::vector<cv::Mat> c;
			// ROS_INFO("SPLITTING");
			// cv::split(img, c);
			// ROS_INFO("SPLITTED");
			// std::vector<cv::Vec4i> lines_std;
    		// lsd_std->detect(c[1], lines_std);
    		// lsd_std->drawSegments(img, lines_std);


cv::Mat img;
cv::Mat pixel_map;

bool img_available = false;

void selectedCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	img_available = true;
	ConvertImage::MsgToMatCopy(msg, img);
}

const uint8_t TEAM_NUMBER = 9;
enum GameStrategy {Defence, Attack, KeepCalmAndCarryOn};
bool broadcastStrategy(GameStrategy strategy)
{
	//Initialize broadcast socket parameters
    static const int broadcast_enable = 1;

    //Initialize UDP broadcast server info
    struct sockaddr_in broadcast_servaddr;
	bzero(&broadcast_servaddr, sizeof(broadcast_servaddr));
	broadcast_servaddr.sin_family = AF_INET;
	broadcast_servaddr.sin_addr.s_addr = inet_addr("255.255.255.255");
	broadcast_servaddr.sin_port = htons(SPL_COACH_MESSAGE_PORT);

    //Open the broadcast socket
   	int broadcast_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (broadcast_socket < 0)
    {
    	std::cerr << "ERROR opening broadcast socket" << std::endl;
    	return false;
    }

    //Set socket timeout to 10ms
    // struct timeval timeout;
    // timeout.tv_sec = 0;
    // timeout.tv_usec = 1000 * 10;
    // setsockopt (broadcast_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));

    //Set the socket to broadcast mode
    setsockopt(broadcast_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    SPLCoachMessage coach_msg;

  	coach_msg.team = TEAM_NUMBER;

    //Send strategy
    memset (coach_msg.message, 0, SPL_COACH_MESSAGE_SIZE);
    switch(strategy)
    {
    	case Defence:
    		strcpy((char*)coach_msg.message, "Defence");
    		break;

    	case Attack:
    	    strcpy((char*)coach_msg.message, "Attack");
    		break;

    	case KeepCalmAndCarryOn:
    		strcpy((char*)coach_msg.message, "Keep calm and carry on");
    		break;
    }
    int n = sendto(broadcast_socket,
                   &coach_msg,
                   sizeof(SPLCoachMessage),
                   0, // No flags
                   (struct sockaddr*)&broadcast_servaddr,
                   sizeof(broadcast_servaddr));

    if(n != sizeof(SPLCoachMessage))
    	return false;

    close(broadcast_socket);

    return true;
}


int main(int argc, char** argv)
{

	//***************************************************
	//************** Read color table *******************
	//***************************************************
	std::cout << "[edinferno_coach.cpp] Loading up and converting the color table... " << std::endl;
	if(!Segmentation::LoadLookupTable("/home/nao/coltablelab.c64"))
	{
		std::cout << "Unable to load lookup table. " << std::endl;
		return 1;
	}
	std::cout << "[edinferno_coach.cpp] Color table loaded and converted. " << std::endl;


	//***************************************************
	//*************** Initialise ROS ********************
	//***************************************************
	ros::init(argc, argv, "edinferno_coach");
	ros::NodeHandle n("~");
	ros::Subscriber sub = n.subscribe("/nao_cams/selected", 1, selectedCameraCallback);
	ros::Publisher seg_pub = n.advertise<sensor_msgs::Image>("/ed_coach/seg", 2);
	ros::Rate r(10);

	sensor_msgs::Image msg;


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
	float searching_head_yaw, searching_head_pitch;

	const int REMEMBER_BALL = 50;
	int ball_not_seen = 0;
	while(ros::ok())
	{
		if(img_available)
		{
			img_available = false;
			// std::cout << "[edinferno_coach.cpp] Segmentation " << std::endl;
			Segmentation::SegmentImage(img, pixel_map);
			// std::cout << "[edinferno_coach.cpp] Detection " << std::endl;
			DetectedBall b = BallDetection::Detect(pixel_map);
			// std::cout << "[edinferno_coach.cpp] Control " << std::endl;
			if(b.detected)
			{
				ball_not_seen = 0;

				cv::circle(img, b.cntr, b.r, cv::Scalar(0,255,255), 2, 8, 0);
				cv::circle(img, b.cntr, 2, cv::Scalar(255,0,0), 2, 8, 0);

				d_yaw = 0.0005 * (b.cntr.x - img.cols / 2);
				if(-0.001 < d_yaw && d_yaw < 0.001) d_yaw = 0;

				head_yaw -= d_yaw; // +ve is looking to the left
				head_yaw = (head_yaw < HEAD_YAW_MIN) ? HEAD_YAW_MIN : (head_yaw > HEAD_YAW_MAX) ? HEAD_YAW_MAX : head_yaw;

				d_pitch = 0.0005 * (b.cntr.y - img.rows / 2);
				if(-0.0001 < d_pitch && d_pitch < 0.0001) d_pitch = 0;

				head_pitch += d_pitch;// +ve is looking down
				head_pitch = (head_pitch < HEAD_PITCH_MIN) ? HEAD_PITCH_MIN : (head_pitch > HEAD_PITCH_MAX) ? HEAD_PITCH_MAX : head_pitch;

				motion.setAngles(joint_names, AL::ALValue::array(head_yaw, head_pitch), 0.8f);
			}
			else
			{
				++ball_not_seen;
			}

			// std::cout << "[edinferno_coach.cpp] ball_not_seen " << ball_not_seen << std::endl;

			if(ball_not_seen >= REMEMBER_BALL)
			{
				std::vector<float> angls = motion.getAngles(joint_names, true);

				float err_yaw = fabs(angls[0] - searching_head_yaw);
				float err_pitch = fabs(angls[1] - searching_head_pitch);
				// std::cout << "[edinferno_coach.cpp] angls " << angls[0] << " " << angls[1] << std::endl;
				// std::cout << "[edinferno_coach.cpp] " << err_yaw << " " << err_pitch << std::endl;
				if(ball_not_seen == REMEMBER_BALL ||
				   (err_yaw < 0.25 && err_pitch < 0.25))
				{

					searching_head_yaw = HEAD_YAW_MIN + (HEAD_YAW_MAX - HEAD_YAW_MIN) * ((rand() % 101) / 100.0f);
					searching_head_pitch = HEAD_PITCH_MIN + (HEAD_PITCH_MAX - HEAD_PITCH_MIN) * ((rand() % 101) / 100.0f);
					// std::cout << "[edinferno_coach.cpp] New Searching Angles " << searching_head_yaw << " " << searching_head_pitch << std::endl;
					motion.setAngles(joint_names, AL::ALValue::array(searching_head_yaw, searching_head_pitch), 0.1f);
				}
			}

    		ConvertImage::MatToMsgCopy(img, "/world", msg);
    		seg_pub.publish(msg);

    		// std::cout << "[edinferno_coach.cpp] Published " << std::endl;
		}
		else
		{
			std::cout << "[edinferno_coach.cpp] No image " << std::endl;
		}
		// std::cout << "[edinferno_coach.cpp] SpinOnce " << std::endl;
		ros::spinOnce();
		// std::cout << "[edinferno_coach.cpp] Sleep " << std::endl;
		r.sleep();
	}
	motion.setStiffnesses(joint_names, AL::ALValue::array(0.0f, 0.0f));

	return 0;
}
