#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include <opencv2/opencv.hpp>
#include <vector>

typedef struct DetectedBall
{
	cv::Point cntr;
	float r;
	bool detected;
} DetectedBall;

class BallDetection
{
public:
	static DetectedBall Detect(cv::Mat& map);

private:
	BallDetection() {};
	~BallDetection() {};

	static cv::Mat gray;
	static std::vector< std::vector<cv::Point> > contours;
};

#endif
