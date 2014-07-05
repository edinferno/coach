#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include <string>
#include <opencv2/core/core.hpp>

enum PixelClass
{
	AnyObject,
	Ball,
	YellowGoal,
	BlueGoal,
	Lines,
	Field,
	Nothing,
	TeamRed,
	TeamBlue
};

// The colors of the pixel classes in BGR
const cv::Scalar ClassColor[] = {
	cv::Scalar(68, 68, 68), 	// AnyObject - grey
	cv::Scalar(0, 128, 255), 	// Ball - orange
	cv::Scalar(0, 255, 255), 	// Goal - yellow
	cv::Scalar(255, 0, 0), 		// Goal - blue
	cv::Scalar(255, 255, 255), 	// Lines - white
	cv::Scalar(0, 255, 0), 		// Field - green
	cv::Scalar(0, 0, 0), 		// Nothing - black
	cv::Scalar(0, 0, 255), 		// TeamRed - red
	cv::Scalar(255, 0, 0) 		// TeamBlue - blue
};

class Segmentation
{
	public:

		static bool LoadLookupTable(std::string filename);
		static void SegmentImage(cv::Mat& img, cv::Mat& map, bool color_img = false);

	private:
		static const int TABLE_SIZE = 64;
		static const int TABLE_LEN = TABLE_SIZE * TABLE_SIZE * TABLE_SIZE;
		static PixelClass classes[TABLE_SIZE][TABLE_SIZE][TABLE_SIZE];

		Segmentation() {};
		~Segmentation() {};

		static inline void YCbCr_to_RGB(unsigned char y, unsigned char cb, unsigned char cr,
		                                unsigned char &r, unsigned char &g, unsigned char &b)
		{
			int r_ = y + ((1436 * ((int)cr - 128)) >> 10);
			int g_ = y - ((354 * ((int)cb - 128) + 732 * ((int)cr - 128)) >> 10);
			int b_ = y + ((1814 * ((int)cb - 128)) >> 10);

			r_ = (r_ < 0) ? 0 : (r_ > 255) ? 255 : r_;
			g_ = (g_ < 0) ? 0 : (g_ > 255) ? 255 : g_;
			b_ = (b_ < 0) ? 0 : (b_ > 255) ? 255 : b_;

			r = (unsigned char) r_;
			g = (unsigned char) g_;
			b = (unsigned char) b_;
		}

		static inline void RGB_to_YCbCr(unsigned char r, unsigned char g, unsigned char b,
		                                unsigned char &y, unsigned char &cb, unsigned char &cr)
		{
			int y_ = (306 * (int)r + 601 * (int)g + 113 * (int)b) >> 10;
			int cr_ = 127 + ((-173 * (int)r - 339 * (int)g + 512 * (int)b) >> 10);
			int cb_ = 127 + ((512 * (int)r - 429 * (int)g - 83 * (int)b) >> 10);

			y_ = (y_ < 0) ? 0 : (y_ > 255) ? 255 : y_;
			cb_ = (cb_ < 0) ? 0 : (cb_ > 255) ? 255 : cb_;
			cr_ = (cr_ < 0) ? 0 : (cr_ > 255) ? 255 : cr_;

			y = (unsigned char) y_;
			cb = (unsigned char) cb_;
			cr = (unsigned char) cr_;
		}

};

#endif
