#include "ball_detection.hpp"

#include "segmentation.hpp"

cv::Mat BallDetection::gray;
std::vector< std::vector<cv::Point> > BallDetection::contours;

DetectedBall BallDetection::Detect(cv::Mat& map)
{
	if(gray.size() != map.size())
	{
		gray = cv::Mat(map.size(), CV_8UC1);
	}

	for(int i = 0; i < map.rows * map.cols; ++i)
	{
		gray.data[i] = ((PixelClass)map.data[i] == Ball) ? 255 : 0;
	}

	cv::findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	double area, max_area = -1, circularity;
	cv::Rect ball_rect;
	DetectedBall result;

	std::cout << "[ball_detection.cpp] Detected contours " << contours.size() << std::endl;
	result.detected = false;
	for(unsigned int i = 0; i < contours.size(); i++)
	{
		// Skip contours which are too short (obviously noise)
		if(contours[i].size() < 10) continue;

		// Skip contours which are too small (obviously noise)
		area = cv::contourArea(contours[i]);
		if(area < 20) continue;

		// Skip contours which are not circular enough
		circularity = 4 * M_PI * area / (contours[i].size() * contours[i].size());
		if(circularity < 0.30f) continue;

		// If the contour fits all criteria and it is the larges so far
		if(area > max_area)
		{
			max_area = area;

			result.detected = true;

			ball_rect = cv::boundingRect(contours[i]);
			result.cntr = cv::Point(ball_rect.x + ball_rect.width / 2, ball_rect.y + ball_rect.height / 2);
			result.r = (ball_rect.width + ball_rect.height + 4) / 4;
		}
	}

	return result;
}
