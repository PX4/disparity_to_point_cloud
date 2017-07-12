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
 * Class to perform a depth map fusion
 *
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

// Need the CameraTriplet-header for informParent()
#include <disparity_to_point_cloud/camera_pair.hpp>
#include <disparity_to_point_cloud/camera_triplet.hpp>

namespace depth_map_fusion {

namespace dynamic_reconfiguration {
  // Evil globals set in depth_map_fusion_node
  extern bool LINE_DETECTION;
  extern int LINE_DET_WEIGHT;
  extern int SUB_MAT_MAX;
}

CameraPair::CameraPair(ros::NodeHandle &nh,
                       CameraTriplet *parent,
                       std::string id,
                       bool is_rotated,
                       bool line_detection,
                       bool is_disabled)
:
  parent(parent),
  is_rotated(is_rotated),
  line_detection(line_detection)
{
  if (is_disabled) {
    return;
  }

  // timestamps = std::vector<ros::Time>(2);
  timestamps = {ros::Time(), ros::Time()};
  std::string best_topic = "/uvc_camera/cam_" + id + "/image_depth";
  std::string sec_topic  = "/uvc_camera/cam_" + id + "/image_rect";
  depth_score_best_sub = nh.subscribe(best_topic, 1, &CameraPair::bestCallback, this);
  depth_score_sec_sub  = nh.subscribe(sec_topic,  1, &CameraPair::secCallback,  this);

  depth_best_pub = nh.advertise<sensor_msgs::Image>("/depth_" + id, 1);
  score_best_pub = nh.advertise<sensor_msgs::Image>("/score_" + id, 1);
  depth_sec_pub = nh.advertise<sensor_msgs::Image>("/depth_sec_" + id, 1);
  score_sec_pub = nh.advertise<sensor_msgs::Image>("/score_sec_" + id, 1);
  score_sub_pub = nh.advertise<sensor_msgs::Image>("/score_sub_" + id, 1);
  score_line_pub = nh.advertise<sensor_msgs::Image>("/score_line_" + id, 1);
  score_comb_pub = nh.advertise<sensor_msgs::Image>("/score_comb_" + id, 1);
}


void CameraPair::bestCallback(const sensor_msgs::ImageConstPtr &msg) {
  imageCallback(msg, true);
}


void CameraPair::secCallback(const sensor_msgs::ImageConstPtr &msg) {
  imageCallback(msg, false);
}


void CameraPair::imageCallback(const sensor_msgs::ImageConstPtr &msg, bool is_best) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Mat depth = disparity->image.clone();
  cv::Mat score;
  if (is_rotated) {
    depth = cropToSquare(depth);
    depth = rotateMat(depth, false);
  }
 
  moveScoreFromDepth(depth, score);

  if (is_rotated) {
    depth *= 2;
  }

  // cv::medianBlur(depth, depth, 5);
  // cv::medianBlur(score, score, 15);

  if (is_best) {
    timestamps[0] = msg->header.stamp;
    depth_best_mat = depth.clone();
    score_best_mat = score.clone();

    publishWithColorDebug(msg, depth_best_mat, depth_best_pub, RAINBOW_WITH_BLACK);
    publishWithColorDebug(msg, score_best_mat, score_best_pub, GRAY_SCALE);
  }
  else {
    timestamps[1] = msg->header.stamp;
    depth_sec_mat = depth.clone();
    score_sec_mat = score.clone();
    publishWithColorDebug(msg, depth_sec_mat, depth_sec_pub, RAINBOW_WITH_BLACK);
    publishWithColorDebug(msg, score_sec_mat, score_sec_pub, GRAY_SCALE);
  }

  informParent(msg);
}

void CameraPair::fusePair(const sensor_msgs::ImageConstPtr &msg) {
  int sub_mat_max = dynamic_reconfiguration::SUB_MAT_MAX;
  score_sub_mat = sub_mat_max - (score_sec_mat - score_best_mat);

  if (dynamic_reconfiguration::LINE_DETECTION) {  // TODO: use line_detection instead
    score_line_mat = score_best_mat.clone();
    if (is_rotated) {
      lineDetection(score_line_mat, 2, 0);
    }
    else {
      lineDetection(score_line_mat, 0, 2);
    }
    int line_det_weight = dynamic_reconfiguration::LINE_DET_WEIGHT;
    score_comb_mat = (score_sub_mat + line_det_weight*score_line_mat) / 2;
  }
  else {
    score_comb_mat = score_sub_mat;
  }
  // cv::medianBlur(hor_pair.score_comb_mat, hor_pair.score_comb_mat, 5);
 
  publishWithColorDebug(msg, score_line_mat, score_line_pub, GRAY_SCALE);
  publishWithColorDebug(msg, score_sub_mat, score_sub_pub, GRAY_SCALE);
  publishWithColorDebug(msg, score_comb_mat, score_comb_pub, GRAY_SCALE);

}


void CameraPair::informParent(const sensor_msgs::ImageConstPtr &msg) {
  parent->fuseIfPossible(msg);
}

}  // depth_map_fusion
