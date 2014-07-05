#ifndef NAO_CAMERA
#define NAO_CAMERA

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

class NaoCamera
{
	public:
		NaoCamera(int _id, std::string _name, cv::Size _resolution = cv::Size(640, 480), float _fps = 15);
    	~NaoCamera();

    	int GetId();
    	std::string GetName();

    	bool SetResolution(cv::Size _resolution);
    	cv::Size GetResolution();

    	void SetFPS(float _fps);
    	float GetFPS();

    	inline bool IsOpened() {return cap->isOpened();}
    	inline void WaitForFrame() { fps_rate->sleep(); };
    	inline void CaptureFrame(cv::Mat& img) { cap->read(img); };

	private:
		cv::VideoCapture* cap;

		int id;
		std::string name;
		cv::Size resolution;
		float fps;
		ros::Rate* fps_rate;
};
#endif
