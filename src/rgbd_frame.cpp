/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */



#include <iostream>
#include <opencv2/core.hpp>

 struct RgbdFrame
 {
 	RgbdFrame();
 	RgbdFrame(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& mask =cv::Mat(), const cv::Mat normals = cv::Mat(), int ID = -1 );

 	virtual ~RgbdFrame();

 	virtual void release();

 	int ID;
 	cv::Mat image, depth, mask, normals;

 };

// object that contains frame data and to be used for odometry purposes
struct OdometryFrame : public RgbdFrame
{
	enum
	{
		CACHE_SRC = 1, CACHE_DST =2, CACHE_ALL = CACHE_SRC + CACHE_DST
	};

	OdometryFrame();
	OdometryFrame(const cv::Mat& image, const cv::Mat& depth, const cv::Mat& mask = cv::Mat(), const cv::Mat& normals = Mat(), int ID = -1);

	virtual void release();

	void releasePyramids();

	std::vector<cv::Mat> pyramidImage;
	std::vector<cv::Mat> pyramidDepth;
	std::vector<cv::Mat> pyramidMask;
	std::vector<cv::Mat> pyramidCloud;
	std::vector<cv::Mat> pyramid_dI_dx;
	std::vector<cv::Mat> pyramid_dI_dy;
	std::vector<cv::Mat> pyramidTexturedMask;
	std::vector<cv::Mat> pyramidNormals;
	std::vector<cv::Mat> pyramidNormalsMask;

	~OdometryFrame();
	
};



