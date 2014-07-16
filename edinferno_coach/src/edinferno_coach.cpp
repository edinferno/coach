#include <ros/ros.h>
#include <convert_image/ConvertImage.hpp>

#include <opencv2/opencv.hpp>

#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alledsproxy.h>

#include <vector>
#include <fstream>

using std::ifstream;
using std::vector;

#include "segmentation.hpp"
#include "ball_detection.hpp"
#include "lsd_opencv.hpp"
#include "game.hpp"

const float MIN_YAW = -0.5, MAX_YAW = 0.5;
const float MIN_PITCH = 0.2, MAX_PITCH = 0.45;
const int YAW_POINTS = 5;
const int PITCH_POINTS = 5;
const float dYAW = (MAX_YAW - MIN_YAW) / (YAW_POINTS - 1);
const float dPITCH = (MAX_PITCH - MIN_PITCH) / (PITCH_POINTS - 1);

const float THRESH_DIST = 20.0f;  // 0.0f;
const float MIN_THRESH_DIST = -THRESH_DIST;
const float MAX_THRESH_DIST = +THRESH_DIST;

vector <vector <vector <cv::Point> > > calibration_matrix(
	PITCH_POINTS,
	vector <vector <cv::Point> >(
		YAW_POINTS,
		vector <cv::Point>(2)
	)
);

cv::Mat img;
cv::Mat pixel_map;

bool img_available = false;

void selectedCameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	img_available = true;
	ConvertImage::MsgToMatCopy(msg, img);
}

GameStrategy decideStrategy(float yaw, float pitch, const cv::Point& pos, cv::Mat& img)
{
	float i = (pitch - MIN_PITCH) / dPITCH;
	float j = (yaw - MIN_YAW) / dYAW;
	// std::cout << "[edinferno_coach.cpp] i = " << i << " j = " << j << std::endl;

	if (j < 0)
	{
		return Attack;
	}
	else if(j > YAW_POINTS - 1)
	{
		return Defence;
	}

	if (i < 0 || i > PITCH_POINTS - 1)
	{
		return KeepCalmAndCarryOn;
	}

	int il = floor(i), ih = ceil(i);
	int jl = floor(j), jh = ceil(j);

	float bl = (1 - (i - il)) * (1 - (j - jl));
	float br = (1 - (i - il)) * (1 - (jh - j));
	float tl = (1 - (ih - i)) * (1 - (j - jl));
	float tr = (1 - (ih - i)) * (1 - (jh - j));

	cv::Point f(0, 0), s(0, 0);
	f += bl * calibration_matrix[il][jl][0];
	f += br * calibration_matrix[il][jh][0];
	f += tl * calibration_matrix[ih][jl][0];
	f += tr * calibration_matrix[ih][jh][0];

	s += bl * calibration_matrix[il][jl][1];
	s += br * calibration_matrix[il][jh][1];
	s += tl * calibration_matrix[ih][jl][1];
	s += tr * calibration_matrix[ih][jh][1];

	line(img, f, s, cv::Scalar(0, 0, 255), 2);

	cv::Point v1, v2;
	v1 = f - s;
	v2 = pos - s;

	float area = v1.x * v2.y - v2.x * v1.y;

	float dist = area / sqrt(v1.x * v1.x + v1.y * v1.y);

	if (dist < MIN_THRESH_DIST) {
		return Attack;
	} else if (dist > MAX_THRESH_DIST) {
		return Defence;
	} else {
		return KeepCalmAndCarryOn;
	}
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
	//*********** Read calibration table ****************
	//***************************************************
	ifstream ifs("/home/nao/linecal.txt");
	for (int i = 0; i < PITCH_POINTS; ++i) {
		for (int j = 0; j < YAW_POINTS; ++j) {
			ifs >> calibration_matrix[i][j][0].x;
			ifs >> calibration_matrix[i][j][0].y;
			ifs >> calibration_matrix[i][j][1].x;
			ifs >> calibration_matrix[i][j][1].y;
		}
	}
	ifs.close();

	// for (int i = 0; i < PITCH_POINTS; ++i) {
	// 	for (int j = 0; j < YAW_POINTS; ++j) {
	// 		std::cout << calibration_matrix[i][j][0].x;
	// 		std::cout << " " << calibration_matrix[i][j][0].y;
	// 		std::cout << "  " << calibration_matrix[i][j][1].x;
	// 		std::cout << " " << calibration_matrix[i][j][1].y;
	// 		std::cout << "   ";
	// 	}
	// 	std::cout << '\n';
	// }

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
	//******** Initialise a NaoQi LED proxy **********
	//***************************************************
	AL::ALLedsProxy leds("127.0.0.1", 9559);

	//
	// std::vector<std::string> grps = leds.listGroups();
	// for (std::vector<std::string>::iterator i = grps.begin(); i != grps.end(); ++i)
	// {
	// 	std::cout << "[edinferno_coach.cpp] " << *i << std::endl;
	// }



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

	GameStrategy strategy;

	while(ros::ok())
	{
		if(img_available)
		{
			img_available = false;
			Segmentation::SegmentImage(img, pixel_map);
			DetectedBall b = BallDetection::Detect(pixel_map);

			strategy = KeepCalmAndCarryOn;

			if(b.detected)
			{
				std::vector<float> angles = motion.getAngles(joint_names, true);
				strategy = decideStrategy(angles[0], angles[1], b.cntr, img);
				switch(strategy) {
					case Attack:
						// std::cout << "[edinferno_coach.cpp] attack " << std::endl;
						leds.fadeRGB("FaceLeds", 0x00FF0000, 1);
						break;
					case Defence:
						// std::cout << "[edinferno_coach.cpp] defence " << std::endl;
						leds.fadeRGB("FaceLeds", 0x0000FF00, 1);
						break;
					case KeepCalmAndCarryOn:
						// std::cout << "[edinferno_coach.cpp] keep calm " << std::endl;
						leds.fadeRGB("FaceLeds", 0x000000FF, 1);
						break;
				}

				ball_not_seen = 0;

				//cv::circle(img, b.cntr, b.r, cv::Scalar(0,255,255), 2, 8, 0);
				//cv::circle(img, b.cntr, 2, cv::Scalar(255,0,0), 2, 8, 0);

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

			// Search for the ball
			if(ball_not_seen >= REMEMBER_BALL)
			{
				std::vector<float> angls = motion.getAngles(joint_names, true);

				float err_yaw = fabs(angls[0] - searching_head_yaw);
				float err_pitch = fabs(angls[1] - searching_head_pitch);

				// Check if new angles should be picked
				if(ball_not_seen == REMEMBER_BALL ||
				   (err_yaw < 0.25 && err_pitch < 0.25))
				{

					searching_head_yaw = HEAD_YAW_MIN + (HEAD_YAW_MAX - HEAD_YAW_MIN) * ((rand() % 101) / 100.0f);
					searching_head_pitch = HEAD_PITCH_MIN + (HEAD_PITCH_MAX - HEAD_PITCH_MIN) * ((rand() % 101) / 100.0f);

					head_yaw = searching_head_yaw;
					head_pitch = searching_head_pitch;

					motion.setAngles(joint_names, AL::ALValue::array(searching_head_yaw, searching_head_pitch), 0.1f);
				}
			}

			std::cout << "[edinferno_coach.cpp] Frame " << ball_not_seen << std::endl;
			Game::BroadcastStrategy(strategy);

    		// ConvertImage::MatToMsgCopy(img, "/world", msg);
    		// seg_pub.publish(msg);
		}


		ros::spinOnce();
		r.sleep();
	}
	motion.setStiffnesses(joint_names, AL::ALValue::array(0.0f, 0.0f));

	return 0;
}
