/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file disparity_to_point_cloud.hpp
 *
 * Class to convert a disparity map in to a point cloud
 *
 * @author Simone Guscetti <simone@px4.io>
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#ifndef __DISPARITY_TO_POINT_CLOUD_HPP__
#define __DISPARITY_TO_POINT_CLOUD_HPP__

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace d2pc {
class Disparity2PCloud {
 private:
  ros::NodeHandle nh_;
  ros::Publisher p_cloud_pub_;
  ros::Subscriber disparity_sub_;
  // TODO import this coefficeint with the calibration file or camera info topic
  // double fx_ = 714.24;
  // double fy_ = 713.5;
  double fx_ = 710.2;
  double fy_ = 709.63;

  // double cx_ = 376;
  // double cy_ = 240;
  double cx_ = 353;
  double cy_ = 216;
  // double base_line_ = 0.043; // odroid stereo
  // double base_line_ = 0.09;    // Omni-stereo
  double base_line_ = 0.068;    // Triplet
  std::string camera_optical_frame_ ;
  cv::Mat Q_;

 public:
  Disparity2PCloud() : nh_("~") {
    printf("Constructor start\n");
    disparity_sub_ =
        nh_.subscribe("/disparity", 1, &Disparity2PCloud::DisparityCb, this);

    p_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1, this);

    // Get Ros parameters
    nh_.param<double>("fx_", fx_, 714.24);
    nh_.param<double>("fy_", fy_, 713.5);
    nh_.param<double>("cx_", cx_, 376);
    nh_.param<double>("cy_", cy_, 240);
    nh_.param<double>("base_line_", base_line_, 0.09);
    nh_.param<std::string>("camera_optical_frame_", camera_optical_frame_, "/camera_optical_frame");

    cv::Mat K = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    cv::Mat distCoeff1 = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat t = (cv::Mat_<double>(3, 1) << -base_line_, 0, 0);
    Q_ = cv::Mat::eye(4, 4, CV_64FC1);

    cv::Mat R1 = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat R2 = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1);
    cv::Mat P2 = cv::Mat::eye(3, 4, CV_64FC1);
    printf("Defining matrices\n");
    cv::Size s;
    s.height = 480;
    s.width = 752;
    cv::stereoRectify(K, distCoeff1, K, distCoeff1, s, R, t, R1, R2, P1, P2, Q_);
    printf("stereoRectify\n");
  };
  // virtual ~Disparity2PCloud();
  void DisparityCb(const sensor_msgs::ImageConstPtr &msg);
};
} /* d2pc */

#endif /* end of include guard: __DISPARITY_TO_POINT_CLOUD_HPP__ */
