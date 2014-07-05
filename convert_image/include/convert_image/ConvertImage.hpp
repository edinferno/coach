#ifndef CONVERT_IMAGE_HPP
#define CONVERT_IMAGE_HPP

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

class ConvertImage
{
private:
    /** Get the address where the vector stores the pointer to the start of its
    * data. */
    static uint8_t** get_vector_start_ptr_ptr(std::vector<uint8_t>& v);

    /** Get the address where the vector stores the pointer to the end of its
    * data. */
    static uint8_t** get_vector_finish_ptr_ptr(std::vector<uint8_t>& v);

    /** Get the address where the vector stores the pointer to the end of the
    * memory that it has allocated for its data (maybe more than size of data). */
    static uint8_t** get_vector_storage_end_ptr_ptr(std::vector<uint8_t>& v);

    /** Corrupt the vector to use external data. Clean its memory to avoid leaks. */
    static void set_vector_data(std::vector<uint8_t>& v, uint8_t* ptr, int size);

    /** If the corrupt data was allocated on the stack it can not be delete'd.
    * Steal the data from the vector so it doesn't attempt to clean it when it
    * goes out of scope. */
    static void unset_vector_data(std::vector<uint8_t>& v);

public:
	//===========================================================================
	//---------------- cv::Mat to sensor_msg::Image conversion ------------------
	//===========================================================================
    static void MatToMsgShareSet(const cv::Mat& mat,
                                 const std::string& frame_id,
                                 sensor_msgs::Image& msg);

    static void MatToMsgShareUnset(sensor_msgs::Image& msg);

    static void MatToMsgCopy(const cv::Mat& mat,
                             const std::string& frame_id,
                             sensor_msgs::Image& msg);

    //===========================================================================
    //---------------- sensor_msg::Image to cv::Mat conversion ------------------
    //===========================================================================

    static void MsgToMatShare(const sensor_msgs::Image::ConstPtr& msg,
                              cv::Mat& mat);

    static void MsgToMatCopy(const sensor_msgs::Image::ConstPtr& msg,
                             cv::Mat& mat);


};


#endif
