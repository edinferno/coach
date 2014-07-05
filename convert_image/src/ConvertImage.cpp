#include "convert_image/ConvertImage.hpp"

#include <cassert>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

uint8_t** ConvertImage::get_vector_start_ptr_ptr(std::vector<uint8_t>& v) {
        return reinterpret_cast<uint8_t**>(&v);
}

uint8_t** ConvertImage::get_vector_finish_ptr_ptr(std::vector<uint8_t>& v) {
        return reinterpret_cast<uint8_t**>(&v) + 1;
}

uint8_t** ConvertImage::get_vector_storage_end_ptr_ptr(std::vector<uint8_t>& v) {
        return reinterpret_cast<uint8_t**>(&v) + 2;
}

void ConvertImage::set_vector_data(std::vector<uint8_t>& v, uint8_t* ptr, int size) {
        // Get vector details
        int v_size = v.size();
        int v_cap = v.capacity();

        uint8_t** ps = get_vector_start_ptr_ptr(v);
        uint8_t** pf = get_vector_finish_ptr_ptr(v);
        uint8_t** pe = get_vector_storage_end_ptr_ptr(v);

        // Debug properties
        assert(NULL != ps);
        assert(NULL != pf);
        assert(NULL != pe);

        assert(*pf - *ps == v_size);
        assert(*pe - *ps == v_cap);

        // Cleanup old memory to avoid leaks
        delete [] *ps;

        // Set new data
        *ps = ptr;
        *pf = ptr + size;
        *pe = *pf;
}

void ConvertImage::unset_vector_data(std::vector<uint8_t>& v) {
        // Get vector details
        uint8_t** ps = get_vector_start_ptr_ptr(v);
        uint8_t** pf = get_vector_finish_ptr_ptr(v);
        uint8_t** pe = get_vector_storage_end_ptr_ptr(v);

        // Unset data
        *ps = NULL;
        *pf = NULL;
        *pe = NULL;
}


void ConvertImage::MatToMsgShareSet(const cv::Mat& mat,
                                    const std::string& frame_id,
                                    sensor_msgs::Image& msg)
{
    assert(mat.type() == CV_8UC3 ||
           mat.type() == CV_8UC1);

    // Initialise the header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;

    // Handle change of resolution
    if(mat.type() == CV_8UC3)
    {
        if(msg.data.size() != 3 * mat.rows * mat.cols)
        {
            msg.encoding = sensor_msgs::image_encodings::BGR8;
            msg.width  = mat.cols;
            msg.height = mat.rows;
            msg.step   = mat.cols * 3;
            msg.is_bigendian = 0;
        }
    }
    else if(mat.type() == CV_8UC1)
    {
        if(msg.data.size() != mat.rows * mat.cols)
        {
            msg.encoding = sensor_msgs::image_encodings::MONO8;
            msg.width  = mat.cols;
            msg.height = mat.rows;
            msg.step   = mat.cols;
            msg.is_bigendian = 0;
        }
    }

    // 'Hack' the data vector such that it points to the image data
    set_vector_data(msg.data, mat.data, mat.rows * mat.cols * 3);
}

void ConvertImage::MatToMsgShareUnset(sensor_msgs::Image& msg)
{
    // 'Unhack' the data vector
    unset_vector_data(msg.data);
}

void ConvertImage::MatToMsgCopy(const cv::Mat& mat,
                                const std::string& frame_id,
                                sensor_msgs::Image& msg)
{
     assert(mat.type() == CV_8UC3 ||
            mat.type() == CV_8UC1);

    // Initialise the header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;

    // Handle change of resolution
    if(mat.type() == CV_8UC3)
    {
        if(msg.data.size() != 3 * mat.rows * mat.cols)
        {
            msg.encoding = sensor_msgs::image_encodings::BGR8;
            msg.width  = mat.cols;
            msg.height = mat.rows;
            msg.step   = mat.cols * 3;
            msg.is_bigendian = 0;

            msg.data.resize(msg.step * msg.height);
        }
    }
    else if(mat.type() == CV_8UC1)
    {
        if(msg.data.size() != mat.rows * mat.cols)
        {
            msg.encoding = sensor_msgs::image_encodings::MONO8;
            msg.width  = mat.cols;
            msg.height = mat.rows;
            msg.step   = mat.cols;
            msg.is_bigendian = 0;

            msg.data.resize(msg.step * msg.height);
        }
    }

    // Copy the image data to the data vector
    memcpy (msg.data.data(), mat.data, msg.data.size());
}

void ConvertImage::MsgToMatShare(const sensor_msgs::Image::ConstPtr& msg,
                                 cv::Mat& mat)
{
    assert(msg->encoding == sensor_msgs::image_encodings::BGR8 ||
           msg->encoding == sensor_msgs::image_encodings::MONO8);

    if(msg->encoding ==  sensor_msgs::image_encodings::BGR8)
    {
        if(3 * mat.rows * mat.cols != (int)msg->data.size())
        {
            mat = cv::Mat(msg->height, msg->width, CV_8UC3);
        }
    }
    else if(msg->encoding ==  sensor_msgs::image_encodings::MONO8)
    {
        if(mat.rows * mat.cols != (int)msg->data.size())
        {
            mat = cv::Mat(msg->height, msg->width, CV_8UC1);
        }
    }

    mat.data = (uchar*)msg->data.data();
}


void ConvertImage::MsgToMatCopy(const sensor_msgs::Image::ConstPtr& msg,
                                cv::Mat& mat)
{
    assert(msg->encoding == sensor_msgs::image_encodings::BGR8 ||
           msg->encoding == sensor_msgs::image_encodings::MONO8);

    if(msg->encoding ==  sensor_msgs::image_encodings::BGR8)
    {
        if(3 * mat.rows * mat.cols != (int)msg->data.size())
        {
            mat = cv::Mat(msg->height, msg->width, CV_8UC3);
        }
    }
    else if(msg->encoding ==  sensor_msgs::image_encodings::MONO8)
    {
        if(mat.rows * mat.cols != (int)msg->data.size())
        {
            mat = cv::Mat(msg->height, msg->width, CV_8UC1);
        }
    }


    memcpy (mat.data, msg->data.data(), msg->data.size());
}

