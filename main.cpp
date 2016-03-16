#include <iostream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <vector>
class CvSlam
{
public:
	std::vector<std::pair<cv::Mat, cv::Mat> > _dataset;

	CvSlam(){}
	~CvSlam(){}

	void readFile(std::string filename);

	
	
	
};

void CvSlam::readFile(std::string filename)
{
	std::ifstream rgbd_pairs(filename.c_str());
	std::string line;
	int num_files = 1;

	while(std::getline(rgbd_pairs, line))
	{
		std::cout << num_files << std::endl;
		std::istringstream iss(line);
		std::string timestamp1, rgb_file, depth_file, timestamp2;
		num_files++;
		if(!(iss>> timestamp1 >> rgb_file>>timestamp2 >> depth_file)) break;
		std::cout<< depth_file << std::endl;
		cv::Mat depth_in = cv::imread(depth_file,CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
		cv::Mat rgb = cv::imread(rgb_file,CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
		cv::Mat depth;
		double min;
		double max;
		cv::minMaxIdx(depth_in, &min, &max);
		// expand your range to 0..255. Similar to histEq();
		depth_in.convertTo(depth,CV_8UC1, 255 / (max-min), -min);
		std::pair<cv::Mat, cv::Mat> curr_file;
		curr_file.first = rgb;
		curr_file.second = depth;
		_dataset.push_back(curr_file);
		cv::namedWindow("Img", cv::WINDOW_AUTOSIZE);
		cv::imshow("Img", depth);
		cv::waitKey(2);
		std::cout<< timestamp2 <<std::endl;	
	}
	rgbd_pairs.close();
}



int main(int argc, char *argv[])
{
	/* code */
	CvSlam slam;
	slam.readFile(argv[1]);

	return 0;
}