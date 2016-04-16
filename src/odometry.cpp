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


 class Odometry
 {
public:
	// class of transformation
	enum
	{
		ROTATION = 1, TRANSLATION = 2, RIGID_BODY_MOTION = 4
	};

	static inline float DEFAULT_MIN_DEPTH(){ return 0.f; } // in meters 
	static inline float DEFAULT_MAX_DEPTH(){ return 4.f; } // in meters
	static inline float DEFAULT_MAX_DEPTH_DIFF(){ return 0.07f; }
	static inline float DEFAULT_MAX_POINTS_PART(){ return 0.07f; }
	static inline float DEFAULT_MAX_TRANSLATION(){ return 0.07f; }
	static inline float DEFAULT_MAX_ROTATION(){ return 15;} // in degrees

	/** Method to compute a transformation from the source frame to the destination one.
	* Some odometry algorithms do not used some data of frames (eg. ICP does not use images).
	* In such case corresponding arguments can be set as empty Mat.
	* The method returns true if all internal computions were possible (e.g. there were enough correspondences,
	* system of equations has a solution, etc) and resulting transformation satisfies some test if it's provided
	* by the Odometry inheritor implementation (e.g. thresholds for maximum translation and rotation).
	* @param srcImage Image data of the source frame (CV_8UC1)
	* @param srcDepth Depth data of the source frame (CV_32FC1, in meters)
	* @param srcMask Mask that sets which pixels have to be used from the source frame (CV_8UC1)
	* @param dstImage Image data of the destination frame (CV_8UC1)
	* @param dstDepth Depth data of the destination frame (CV_32FC1, in meters)
	* @param dstMask Mask that sets which pixels have to be used from the destination frame (CV_8UC1)
	* @param Rt Resulting transformation from the source frame to the destination one (rigid body motion):
	dst_p = Rt * src_p, where dst_p is a homogeneous point in the destination frame and src_p is
	homogeneous point in the source frame,
	Rt is 4x4 matrix of CV_64FC1 type.
	* @param initRt Initial transformation from the source frame to the destination one (optional)
	*/

	bool compute(const cv::Mat& srcImage, const cv::Mat& srcDepth, const cv::Mat& srcMask
				const cv::Mat& dstImage, const cv::Mat& dstDepth,
				const cv::Mat& dstMask, cv::Mat& Rt, const cv::Mat& initRt = cv::Mat()) const;

	/** One more method to compute a transformation from the source frame to the destination one.
	* It is designed to save on computing the frame data (image pyramids, normals, etc.).
	*/

	bool compute(cv::Ptr<OdometryFrame>& srcFrame, cv::Ptr<OdometryFrame>& dstFrame,
				cv::Mat& Rt, const cv::Mat& initRt = cv::Mat()) const;



 };















