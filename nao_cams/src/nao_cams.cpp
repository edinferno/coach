#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <convert_image/ConvertImage.hpp>


#include "nao_cams/SetPublishMode.h"
#include "nao_cams/SelectCamera.h"
#include "nao_cams/NaoCamera.hpp"

enum PublishMode {TOP, BOTTOM, BOTH, SELECTED, RECORD};
PublishMode pub_mode = SELECTED;

NaoCamera* top_cam;
NaoCamera* bot_cam;
NaoCamera* sel_cam;

bool openVideoStreams(std::string& input_mode)
{
    ROS_DEBUG("Input mode: %s", input_mode.c_str());

    if (input_mode.compare("capture") == 0)
    {
        top_cam = new NaoCamera(0, "top_camera");
        if (!top_cam->IsOpened())
        {
            ROS_ERROR("Unable to open top camera (/dev/video0).");
            return false;
        }
        bot_cam = new NaoCamera(1, "bottom_camera");
        if (!bot_cam->IsOpened())
        {
            ROS_ERROR("Unable to open bottom camera (/dev/video1).");
            return false;
        }
    }
    else if (input_mode.compare("file") == 0)
    {
        top_cam = new NaoCamera(-1, "/home/nao/video/top/top_cam%04d.png", cv::Size(640, 480), 5);
         if (!top_cam->IsOpened())
        {
            ROS_ERROR("Unable to open top camera (/home/nao/video/top/top_cam%%04d.png).");
            return false;
        }
        bot_cam = new NaoCamera(-2, "/home/nao/video/bottom/bot_cam%04d.png", cv::Size(640, 480), 5);
        if (!bot_cam->IsOpened())
        {
            ROS_ERROR("Unable to open bottom camera (/home/nao/video/bottom/bot_cam%%04d.png).");
            return false;
        }
    }
    else
    {
        ROS_FATAL("Unsupported input mode: %s", input_mode.c_str());
        return false;
    }

    // Set the top camera to be selected by default
    sel_cam = top_cam;

    return true;
}

//===========================================================================
//----------------============ Service callbacks ============----------------
//===========================================================================


bool setBottomCameraInfo(sensor_msgs::SetCameraInfo::Request  &req,
                         sensor_msgs::SetCameraInfo::Response &res)
{
    std::stringstream status;

    if(!bot_cam->SetResolution(cv::Size(req.camera_info.width, req.camera_info.height)))
    {
        status << "Could not set bottom camera resolution to "
               << req.camera_info.width << "x"
               << req.camera_info.height << ".";
        res.status_message = status.str();
        return false;
    }

	return true;
}

bool setTopCameraInfo(sensor_msgs::SetCameraInfo::Request  &req,
                      sensor_msgs::SetCameraInfo::Response &res)
{
     std::stringstream status;

    if(!top_cam->SetResolution(cv::Size(req.camera_info.width, req.camera_info.height)))
    {
        status << "Could not set top camera resolution to "
               << req.camera_info.width << "x"
               << req.camera_info.height << ".";
        res.status_message = status.str();
        return false;
    }

    return true;
}

bool setPublishMode(nao_cams::SetPublishMode::Request  &req,
                    nao_cams::SetPublishMode::Response &res)
{
    pub_mode = (PublishMode)req.pub_mode;

    switch(pub_mode)
    {
        case TOP:
            top_cam->SetFPS(req.fps);
            break;

        case BOTTOM:
            bot_cam->SetFPS(req.fps);
            break;

        case RECORD:
        case BOTH:
            top_cam->SetFPS(req.fps);
            bot_cam->SetFPS(req.fps);
            break;

        case SELECTED:
            sel_cam->SetFPS(req.fps);
            break;
    }

    return true;
}

bool selectCamera(nao_cams::SelectCamera::Request  &req,
                  nao_cams::SelectCamera::Response &res)
{
    sel_cam = (req.camera_id == 0) ? top_cam : bot_cam;

    sel_cam->SetFPS(req.fps);
    if(!sel_cam->SetResolution(cv::Size(req.width, req.height)))
        return false;

    return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nao_cams");
	ros::NodeHandle n("~");

    ROS_INFO("[nao_cams] node started");

    ROS_INFO("[nao_cams] openning video streams");
	std::string input_mode;
    //To override from command line use:
    // $ rosrun nao_cams nao_cams _input_mode:="file"
    n.param("input_mode", input_mode, std::string("capture"));
    if(!openVideoStreams(input_mode))
    {
        ROS_FATAL("[nao_cams] Unable to open video streams");
        return 1;
    }

    ROS_INFO("[nao_cams] advertising services");
	// SetCameraInfo service servers
    ros::ServiceServer top_cam_info_srvr =
        n.advertiseService("set_bottom_camera_info", setBottomCameraInfo);

    ros::ServiceServer bot_cam_info_srvr =
        n.advertiseService("set_top_camera_info", setTopCameraInfo);

    // FPS and Publish Mode server
    ros::ServiceServer set_pub_mode_srvr =
        n.advertiseService("set_publish_mode", setPublishMode);

    // Select camera server
    ros::ServiceServer select_camera_srvr =
        n.advertiseService("select_camera", selectCamera);

    ROS_INFO("[nao_cams] advetising topics");
    ros::Publisher top_cam_pub = n.advertise<sensor_msgs::Image>("top", 2);
	ros::Publisher bot_cam_pub = n.advertise<sensor_msgs::Image>("bottom", 2);
	ros::Publisher sel_cam_pub = n.advertise<sensor_msgs::Image>("selected", 2);


    cv::Mat top_img, bot_img, sel_img;
    sensor_msgs::Image top_msg, bot_msg, sel_msg;


    int frame_no = 0;

    ROS_INFO_STREAM("[nao_cams] publishing 640x480 images on " << sel_cam_pub.getTopic() <<  " @ 15fps");
    ROS_INFO("[nao_cams] node is running...");

    while(ros::ok())
    {
        switch(pub_mode)
        {
            case TOP:
                top_cam->CaptureFrame(top_img);
                ConvertImage::MatToMsgShareSet(top_img, top_cam->GetName(), top_msg);
                top_cam_pub.publish(top_msg);
                ConvertImage::MatToMsgShareUnset(top_msg);
                break;

            case BOTTOM:
                bot_cam->CaptureFrame(bot_img);
                ConvertImage::MatToMsgShareSet(bot_img, bot_cam->GetName(), bot_msg);
                bot_cam_pub.publish(bot_msg);
                ConvertImage::MatToMsgShareUnset(bot_msg);
                break;

            case BOTH:
                top_cam->CaptureFrame(top_img);
                ConvertImage::MatToMsgShareSet(top_img, top_cam->GetName(), top_msg);
                top_cam_pub.publish(top_msg);
                ConvertImage::MatToMsgShareUnset(top_msg);

                bot_cam->CaptureFrame(bot_img);
                ConvertImage::MatToMsgShareSet(bot_img, bot_cam->GetName(), bot_msg);
                bot_cam_pub.publish(bot_msg);
                ConvertImage::MatToMsgShareUnset(bot_msg);

                break;

            case SELECTED:
                sel_cam->CaptureFrame(sel_img);
                ConvertImage::MatToMsgShareSet(sel_img, sel_cam->GetName(), sel_msg);
                sel_cam_pub.publish(sel_msg);
                ConvertImage::MatToMsgShareUnset(sel_msg);
                break;

            case RECORD:
                // top_cam->CaptureFrame(top_img);
                // std::stringstream top_name;
                // top_name << "/home/nao/video/top/top_cam" << std::setfill('0') << std::setw(6) << frame_no << ".png";
                // imwrite(top_name.str(), top_img);

                bot_cam->CaptureFrame(bot_img);
                std::stringstream bot_name;
                bot_name << "/home/nao/video/bottom/bot_cam" << std::setfill('0') << std::setw(6) << frame_no << ".png";
                imwrite(bot_name.str(), bot_img);

                frame_no++;
                break;
        }


        ros::spinOnce();

        switch(pub_mode)
        {
            case RECORD:
            case BOTH:
            case TOP:
                top_cam->WaitForFrame();
                break;

            case BOTTOM:
                bot_cam->WaitForFrame();
                break;

            case SELECTED:
                sel_cam->WaitForFrame();
                break;
        }
    }
	return 0;
}
