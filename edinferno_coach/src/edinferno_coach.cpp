#include <ros/ros.h>
#include <convert_image/ConvertImage.hpp>

#include <opencv2/opencv.hpp>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>

#include "segmentation.hpp"
#include "ball_detection.hpp"
#include "lsd_opencv.hpp"

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
    motion.wakeUp();
    // Make sure the head is stiff to be able to move it
    motion.setStiffnesses(joint_names, AL::ALValue::array(1.0f, 1.0f));

    // Make sure the head is in initial position
    float head_yaw = 0.0f, head_pitch = 0.0f;
    float d_yaw, d_pitch;
	motion.setAngles(joint_names, AL::ALValue::array(head_yaw, head_pitch), 0.1f);

	while(ros::ok())
	{
		if(img_available)
		{
			img_available = false;
			std::cout << "[edinferno_coach.cpp] Segmentation " << std::endl;
			Segmentation::SegmentImage(img, pixel_map);
			std::cout << "[edinferno_coach.cpp] Detection " << std::endl;
			DetectedBall b = BallDetection::Detect(pixel_map);
			std::cout << "[edinferno_coach.cpp] Control " << std::endl;
			if(b.detected)
			{
				cv::circle(img, b.cntr, b.r, cv::Scalar(0,255,255), 2, 8, 0);
				cv::circle(img, b.cntr, 2, cv::Scalar(255,0,0), 2, 8, 0);

				d_yaw = 0.0005 * (b.cntr.x - img.cols / 2);
				if(-0.001 < d_yaw && d_yaw < 0.001) d_yaw = 0;

				head_yaw -= d_yaw; // +ve is looking to the left
				head_yaw = (head_yaw < -1.5f) ? -1.5f : (head_yaw > 1.5f) ? 1.5f : head_yaw;

				d_pitch = 0.0005 * (b.cntr.y - img.rows / 2);
				if(-0.0001 < d_pitch && d_pitch < 0.0001) d_pitch = 0;

				head_pitch += d_pitch;// +ve is looking down
				head_pitch = (head_pitch < -1.0f) ? -1.0f : (head_pitch > 1.0f) ? 1.0f : head_pitch;

				motion.setAngles(joint_names, AL::ALValue::array(head_yaw, head_pitch), 0.8f);
			}

    		ConvertImage::MatToMsgCopy(img, "/world", msg);
    		seg_pub.publish(msg);

    		std::cout << "[edinferno_coach.cpp] Published " << std::endl;
		}
		else
		{
			std::cout << "[edinferno_coach.cpp] No image " << std::endl;
		}
		std::cout << "[edinferno_coach.cpp] SpinOnce " << std::endl;
		ros::spinOnce();
		std::cout << "[edinferno_coach.cpp] Sleep " << std::endl;
		r.sleep();
	}

	motion.setStiffnesses(joint_names, AL::ALValue::array(0.0f, 0.0f));

	return 0;
}
