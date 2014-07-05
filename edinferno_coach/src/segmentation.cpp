#include "segmentation.hpp"

#include <iomanip>
#include <cassert>
#include <fstream>
#include <iostream>

PixelClass Segmentation::classes[TABLE_SIZE][TABLE_SIZE][TABLE_SIZE];

const int Segmentation::TABLE_SIZE;
const int Segmentation::TABLE_LEN;

bool Segmentation::LoadLookupTable(std::string filename)
{
	std::ifstream tbl_file;
	tbl_file.open(filename.c_str(), std::ios::binary);

	if(!tbl_file.is_open())
	{
		return false;
	}

	// Make sure the lookup table is empty
	for(int i = 0; i < TABLE_LEN; ++i)
	{
		((PixelClass*)classes)[i] = Nothing;
	}

	// the current color class in the lookup table
	char cur_class;

	// the number of bytes in one sequence that have the same color class
	int cur_len;

	// the position in the lookup table
	int pos = 0;

	// std::cout << "test " << std::hex << 30441 << std::endl;

	while (pos < TABLE_LEN)
	{
		tbl_file.read((char*)&cur_len, sizeof(int));
		assert(cur_len != 0);

		tbl_file >> cur_class;

		for (unsigned int l = 0; l < cur_len; ++l)
		{
			int idx = pos + l;
			int y = (idx / TABLE_SIZE / TABLE_SIZE) << 2;
			int cb = ((idx / TABLE_SIZE) % TABLE_SIZE) << 2;
			int cr = (idx % TABLE_SIZE) << 2;

			unsigned char r, g, b;
			for(int i = 0; i < 4; ++i)
			{
				for(int j = 0; j < 4; ++j)
				{
					for(int k = 0; k < 4; ++k)
					{
						Segmentation::YCbCr_to_RGB((unsigned char)y + i,
			                           (unsigned char)cb + j,
			                           (unsigned char)cr + k,
			                           r, g, b);

						classes[r >> 2][g >> 2][b >> 2] = (PixelClass)cur_class;
					}
				}
			}

			assert(pos + l < TABLE_LEN);
		}

		pos += cur_len;
	}

	return true;
}

void Segmentation::SegmentImage(cv::Mat& img, cv::Mat& map, bool color_img)
{
	assert(img.type() == CV_8UC3);

	if(map.size() != img.size())
	{
		map = cv::Mat(img.size(), CV_8UC1);
	}

	for(int i = 0; i < img.rows; ++i)
	{
		for(int j = 0; j < img.cols; ++j)
		{
			int idx = 3 * (i * img.cols + j);
			unsigned char r = img.data[idx + 2] >> 2;
			unsigned char g = img.data[idx + 1] >> 2;
			unsigned char b = img.data[idx] >> 2;

			map.data[i * img.cols + j] = classes[r][g][b];

			if(color_img)
			{
				int c = classes[r][g][b];
				img.data[idx] = ClassColor[c][0];
				img.data[idx + 1] = ClassColor[c][1];
				img.data[idx + 2] = ClassColor[c][2];
			}

		}
	}
}
