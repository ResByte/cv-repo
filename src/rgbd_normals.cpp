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

class RgbdNormals
{
public:
	enum RGBD_NORMALS_METHOD
	{
		RGBD_NORMALS_METHOD_FALS, RGBD_NORMALS_METHOD_LINEMOD, RGBD_NORMALS_METHOD_SRI
	};

	RgbdNormals()
		:
			rows_(0),
			cols_(0),
			depth_(0),
			K_(cv::Mat()),
			window_size_(0),
			method_(RGBD_NORMALS_METHOD_FALS),
			rgbd_normals_impl_(0)
	{
	}

	//constructor
	RgbdNormals(int rows, int depth, cv::InputArray K, int window_size = 5, int method = 
		RGBD_NORMALS_METHOD_FALS);

	//destructor
	~RgbdNormals();

	// given a set of 3d points in a depth image, compute the normals at each point.
	void operator()(cv::InputArray points, cv::OutputArray normals) const;

	// initialize data
	void initialize() const;

	int getRows() const
	{
		return rows_;
	}

	int setRows(int val)
	{
		rows_ = val;
	}

	int getCols() const
	{
		return cols_;
	}

	void setCols(int val)
	{
		cols_ = val
	}

	int getWindowSize() const
	{
		return window_size_;
	}

	void setWindowSize(int val)
	{
		window_size_ = val;
	}

	int getDepth() const
	{
		return depth_;
	}

	void setDepth(int val)
	{
		depth_ = val;
	}

	cv::Mat getK() const
	{
		return K_;
	}

	void setK(const cv::Mat &val)
	{
		K_ = val;
	}

	int getMethod() const
	{
		return method_;
	}

	void setMethod(int val)
	{
		method_ = val;
	}

protected:
	void initialize_normals_impl(int rows, int cols, int depth, const cv::Mat & K, int window_size, int method ) const;

	int rows_, cols_, depth_;
	cv::Mat K_;
	int window_size_;
	int method_;
	mutable void* rgbd_normals_impl_;
};


















