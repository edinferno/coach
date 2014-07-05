#include "nao_cams/NaoCamera.hpp"

NaoCamera::NaoCamera(int _id, std::string _name, cv::Size _resolution, float _fps)
{
	id = _id;
	name = _name;
	fps_rate = NULL;

	if(id >= 0)
	{
		cap = new cv::VideoCapture(id);
	}
	else
	{
		cap = new cv::VideoCapture(name);
	}
	SetResolution(_resolution);
	SetFPS(_fps);

}

NaoCamera::~NaoCamera()
{
	if(cap != NULL) delete cap;
	if(fps_rate != NULL) delete fps_rate;
}

int NaoCamera::GetId()
{
	return id;
}

std::string NaoCamera::GetName()
{
	return name;
}

bool NaoCamera::SetResolution(cv::Size _resolution)
{
	if(cap->set(CV_CAP_PROP_FRAME_HEIGHT, _resolution.height))
    {
        return false;
    }
    if(cap->set(CV_CAP_PROP_FRAME_WIDTH, _resolution.width))
    {
        return false;
    }

    resolution = _resolution;
    return true;
}

cv::Size NaoCamera::GetResolution()
{
	return resolution;
}

void NaoCamera::SetFPS(float _fps)
{
	fps = _fps;

	if(fps_rate != NULL)
		delete fps_rate;

    fps_rate = new ros::Rate(fps);
}

float NaoCamera::GetFPS()
{
	return fps;
}
