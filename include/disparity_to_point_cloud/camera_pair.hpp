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
 * @file depth_map_fusion.hpp
 *
 * Class for a camera pair from uvc_ros_driver
 *
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#ifndef __CAMERA_PAIR_HPP__
#define __CAMERA_PAIR_HPP__

#include <deque>
#include <numeric>
#include <string>


#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <disparity_to_point_cloud/common.h>

namespace depth_map_fusion {

// Forward declare CameraTriplet because of the parent pointer
class CameraTriplet;

class CameraPair {
 public:
  CameraPair(ros::NodeHandle &nh,
             CameraTriplet *parent,
             std::string id,
             bool is_rotated=false,
             bool line_detection=true,
             bool is_disabled=false);

  // Normal callback, image contains depth and score 
  void bestCallback(const sensor_msgs::ImageConstPtr &msg);

  // 'Second-best' callback, image contains depth and score of second best match
  void secCallback(const sensor_msgs::ImageConstPtr &msg);
  
  // Same as bestCallback, secCallback
  void imageCallback(const sensor_msgs::ImageConstPtr &msg, bool is_best);

  void fusePair(const sensor_msgs::ImageConstPtr &msg);

  // Informs the parent-pointer that an image was received
  void informParent(const sensor_msgs::ImageConstPtr &msg);

  bool is_rotated;
  bool line_detection;
  std::vector<ros::Time> timestamps;
  cv::Mat depth_best_mat;
  cv::Mat score_best_mat;
  cv::Mat depth_sec_mat;
  cv::Mat score_sec_mat;
  cv::Mat score_sub_mat;
  cv::Mat score_line_mat;
  cv::Mat score_comb_mat;
  ros::Subscriber depth_score_best_sub;
  ros::Subscriber depth_score_sec_sub;
  ros::Publisher depth_best_pub;
  ros::Publisher depth_sec_pub;
  ros::Publisher score_best_pub;
  ros::Publisher score_sec_pub;
  ros::Publisher score_sub_pub;
  ros::Publisher score_line_pub;
  ros::Publisher score_comb_pub;
  CameraTriplet *parent;
};

}  // depth_map_fusion

#endif  // __CAMERA_PAIR_HPP__
