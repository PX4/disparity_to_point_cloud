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
 * @file depth_map_fusion.cpp
 *
 * Class to perform a depth map fusion
 *
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#include "disparity_to_point_cloud/depth_map_fusion.hpp"

namespace depth_map_fusion {

// Callbacks
void DepthMapFusion::DisparityCb1(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Mat rot_mat = rotateMat(disparity->image);
  rect2 = disparity->image.clone();
  cropped_depth_1_ = cropToSquare(rot_mat, -offset_x_, -offset_y_);

  moveScoreFromDepth(cropped_depth_1_, cropped_score_1_);

  publishWithColor(msg, cropped_depth_1_, cropped_depth_1_pub_,
                   RAINBOW_WITH_BLACK);
  publishWithColor(msg, cropped_score_1_, cropped_score_1_pub_, GRAY_SCALE);
}

void DepthMapFusion::DisparityCb2(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cropped_depth_2_ = cropToSquare(disparity->image, offset_x_, offset_y_);

  moveScoreFromDepth(cropped_depth_2_, cropped_score_2_);

  publishWithColor(msg, cropped_depth_2_, cropped_depth_2_pub_, RAINBOW_WITH_BLACK);
  publishWithColor(msg, cropped_score_2_, cropped_score_2_pub_, GRAY_SCALE);
}

void DepthMapFusion::MatchingScoreCb1(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  rect1 = disparity->image.clone();
  cv::Mat rot_mat = rotateMat(disparity->image);
  cropped_depth_1_second_ = cropToSquare(rot_mat, -offset_x_, -offset_y_);
  
  moveScoreFromDepth(cropped_depth_1_second_, cropped_score_1_second_);

  publishWithColor(msg, cropped_depth_1_second_, cropped_depth_1_second_pub_, RAINBOW_WITH_BLACK);
  publishWithColor(msg, cropped_score_1_second_, cropped_score_1_second_pub_, GRAY_SCALE);  
}

void DepthMapFusion::MatchingScoreCb2(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cropped_depth_2_second_ = cropToSquare(disparity->image, offset_x_, offset_y_);
  
  moveScoreFromDepth(cropped_depth_2_second_, cropped_score_2_second_);

  publishWithColor(msg, cropped_depth_2_second_, cropped_depth_2_second_pub_, RAINBOW_WITH_BLACK);
  publishWithColor(msg, cropped_score_2_second_, cropped_score_2_second_pub_, GRAY_SCALE);  
 
  publishFusedDepthMap(msg);

  // publishStereoSGBM(msg);
}

void DepthMapFusion::publishStereoSGBM(const sensor_msgs::ImageConstPtr &msg) {
  // cv::Mat disp;
  // int block_size = 3;
  // double minVal; double maxVal;
  // cv::Mat output;

  // auto sgbm = cv::StereoSGBM::create(0, 16, block_size * block_size);
  // sgbm->setP1(8 * block_size * block_size);
  // sgbm->setP2(32 * block_size * block_size);
  // sgbm->setMinDisparity(0);
  // sgbm->setNumDisparities(16);
  // sgbm->setUniquenessRatio(50);
  // sgbm->setSpeckleWindowSize(0);
  // sgbm->setSpeckleRange(100);
  // sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
  // sgbm->compute(rect2, rect1, disp);
  // minMaxLoc( disp, &minVal, &maxVal );
  // // std::cout << "StereSGBM (min, max): " << minVal << " " << maxVal << std::endl;
  // disp.convertTo(output, CV_8U);
  // // imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
  // publishWithColor(msg, output, stereoSGBM_pub_, RAINBOW_WITH_BLACK, "mono8");

  // auto sbm = cv::StereoBM::create(0, 21);
  // sbm->setMinDisparity(0);
  // sbm->setNumDisparities(16);
  // sbm->setUniquenessRatio(10);
  // sbm->setSpeckleWindowSize(1);
  // sbm->setSpeckleRange(100);
  // sbm->compute(rect2, rect1, disp);
  // minMaxLoc( disp, &minVal, &maxVal );
  // disp.convertTo(output, CV_8U);
  // publishWithColor(msg, output, stereoBM_pub_, RAINBOW_WITH_BLACK, "mono8");

  // // cv::Mat out = cv::Mat::zeros(rect2.size(), rect2.type());
  // // rect2(cv::Rect(0,1, rect2.cols,rect2.rows-1)).copyTo(out(cv::Rect(0,0,rect2.cols,rect2.rows-1)));
  // // rect2 = out;
  // sbm->compute(rect2, rect1, disp);
  // minMaxLoc( disp, &minVal, &maxVal );
  // disp.convertTo(output, CV_8U);
  // publishWithColor(msg, output, stereoBM_fixed_pub_, RAINBOW_WITH_BLACK, "mono8");
}

// Fuses together the last depthmaps and publishes
// If messages don't arrive sequencially they may have different time stamps
void DepthMapFusion::publishFusedDepthMap(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  disparity->image = cropToSquare(disparity->image);

  if (cropped_depth_1_.empty() || cropped_depth_1_second_.empty() ||
      cropped_depth_2_.empty() || cropped_depth_2_second_.empty()) {
    return;  // Have not recieved all depth maps and scores
  }

  cropped_depth_combined_ = cropped_depth_1_.clone();
  cropped_score_combined_ = cropped_score_1_.clone();
  // For each pixel, calculate a new distance
  for (int i = 0; i < cropped_depth_2_.rows; ++i) {
    for (int j = 0; j < cropped_depth_2_.cols; ++j) {
      cropped_depth_combined_.at<unsigned char>(i, j) = getFusedDistance(i, j);
      unsigned char combined_confidence = std::min(cropped_score_1_.at<unsigned char>(i, j),
                                                   cropped_score_2_.at<unsigned char>(i, j));
      cropped_score_combined_.at<unsigned char>(i, j) = combined_confidence;
    }
  }
  cv::medianBlur(cropped_depth_combined_, cropped_depth_combined_, 5);

  publishWithColor(msg, cropped_depth_combined_, grad_pub_, RAINBOW_WITH_BLACK);
  publishWithColor(msg, cropped_score_combined_, cropped_score_combined_pub_, GRAY_SCALE);

  sensor_msgs::Image fused_image;
  disparity->toImageMsg(fused_image);
  fused_pub_.publish(fused_image);

  publishWithColor(msg, 100 - (cropped_score_1_second_ - cropped_score_1_), cropped_score_1_diff_pub_, GRAY_SCALE);
  publishWithColor(msg, 100 - (cropped_score_2_second_ - cropped_score_2_), cropped_score_2_diff_pub_, GRAY_SCALE);

  // Print out the error between the two score images
  // int curr_error = cv::norm(cropped_score_1_, cropped_depth_2_);
  // previous_errors_. push_front(curr_error);
  // if (previous_errors_.size() > 10) {
  //   previous_errors_.pop_back();
  //   double sum_error = std::accumulate(previous_errors_.begin(),
  //   previous_errors_.end(), 0);
  //   std::cout << sum_error / 10.0 << std::endl;
  // }
}

// Returns the fused depth for pixel (i,j)
int DepthMapFusion::getFusedDistance(int i, int j) {
  int dist1 = cropped_depth_1_.at<unsigned char>(i, j);
  int dist2 = cropped_depth_2_.at<unsigned char>(i, j);
  int score1 = cropped_score_1_.at<unsigned char>(i, j);
  int score2 = cropped_score_2_.at<unsigned char>(i, j);
  int grad1 = 0; //cropped_score_1_grad_.at<unsigned char>(i, j);
  int grad2 = 0; //cropped_score_2_grad_.at<unsigned char>(i, j);

  // score1 = dist1 ^ 7;
  // score2 = dist2 ^ 7;

  score1 = 100 - (cropped_score_2_second_.at<unsigned char>(i, j) - score1);
  score2 = 100 - (cropped_score_1_second_.at<unsigned char>(i, j) - score2);

  // Choose which fusion function to use
  return gradFilter(dist1, dist2, score1, score2, grad1, grad2);
}

int DepthMapFusion::weightedAverage(int dist1, int dist2, int score1,
                                    int score2) {
  int weight1 = std::max(0.01, 1.0 - (0.5 * score1));
  int weight2 = std::max(0.01, 1.0 - (0.5 * score2));
  return (weight1 * dist1 + weight2 * dist2) / (weight1 + weight2);
}

int DepthMapFusion::maxDist(int dist1, int dist2, int score1, int score2) {
  // Lower value means larger distance
  return std::min(dist1, dist2);
}

int DepthMapFusion::maxDistUnlessBlack(int dist1, int dist2, int score1,
                                       int score2) {
  if (dist1 == 0 || dist2 == 0) {
    return std::max(dist1, dist2);
  }
  return std::min(dist1, dist2);
}

int DepthMapFusion::betterScore(int dist1, int dist2, int score1, int score2) {
  if (score1 < score2) {
    // A small score is better
    return dist1;
  }
  return dist2;
}

int DepthMapFusion::onlyGood1(int dist1, int dist2, int score1, int score2) {
  if (score2 < 50) {
    // A small score is better
    return dist2;
  }
  return 0;
}

int DepthMapFusion::onlyGoodAvg(int dist1, int dist2, int score1, int score2) {
  if (score1 < 100 && score2 < 100) {
    return (dist1 + dist2) / 2;
  }
  return 0;
}

int DepthMapFusion::overlap(int dist1, int dist2, int score1, int score2) {
  if (score1 < score2 && score1 < 20) {
    // A small score is better
    return 150;
  } else if (score2 < score1 && score2 < 20) {
    return 255;
  }
  return 0;
}

int DepthMapFusion::blackToWhite(int dist1, int dist2, int score1, int score2) {
  return 255 - score1;
}

int DepthMapFusion::gradFilter(int dist1, int dist2, int score1, int score2,
                               int grad1, int grad2) {
  int thres = 180;
  int tooClose = 230;

  float relative_diff = float(dist1) / float(dist2);

  if (score1 < score2 && score1 < thres && dist1 < tooClose && dist1!=0 && dist2!=0) {
    return dist1;
  } else if (score2 < score1 && score2 < thres && dist2 < tooClose && dist1!=0 && dist2!=0) {
    return dist2;
  } else if (0.8 < relative_diff && relative_diff < 1.25 &&
             score1 < 1.25 * thres && score2 < 1.25 * thres && dist1!=0 && dist2!=0) {
    return float(dist1 + dist2) / 2.0;
  }
  return 0;
}

// void erode() {
//   cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
//   cropped_score_1_ = cropToSquare(disparity->image, offset_x_, offset_y_);
//   cv::GaussianBlur(cropped_score_1_, cropped_score_1_grad_, cv::Size(5,5),
//   10.0);
//   threshold(cropped_score_1_grad_, cropped_score_1_grad_, 50, 255, 1);
//   cv::GaussianBlur(cropped_score_1_grad_, cropped_score_1_grad_,
//   cv::Size(11,11), 10.0);
//   cv::Sobel(cropped_score_1_grad_, cropped_score_1_grad_, -1, 0, 2, 7, 0.03);
//   threshold(cropped_score_1_grad_, cropped_score_1_grad_, 50, 255, 0);

//   cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT,
//   cv::Size(40,1));
//   // Apply morphology operations
//   cv::erode(cropped_score_1_grad_, cropped_score_1_grad_,
//   horizontalStructure, cv::Point(-1, -1));
//   cv::dilate(cropped_score_1_grad_, cropped_score_1_grad_,
//   horizontalStructure, cv::Point(-1, -1));
//   cv::GaussianBlur(cropped_score_1_grad_, cropped_score_1_grad_,
//   cv::Size(21,21), 10.0);
//   cropped_score_1_grad_ = cropped_score_1_ + cropped_score_1_grad_;
// }

// cv::GaussianBlur(cropped_score_1_, cropped_score_1_grad_, cv::Size(5,5),
// 10.0);
// threshold(cropped_score_1_grad_, cropped_score_1_grad_, 50, 255, 1);
// cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT,
// cv::Size(1,20));
// // Apply morphology operations
// cv::erode(cropped_score_1_grad_, cropped_score_1_grad_, horizontalStructure,
// cv::Point(-1, -1));

// cv::Mat kernelH(5, 5, CV_32F, float(0));
// kernelH.at<float>(0,2) = 0.5f;
// kernelH.at<float>(1,2) = 0.5f;
// kernelH.at<float>(3,2) = 0.5f;
// kernelH.at<float>(4,2) = 0.5f;
// kernelH.at<float>(2,0) = -0.5f;
// kernelH.at<float>(2,1) = -0.5f;
// kernelH.at<float>(2,2) = 1.0f;
// kernelH.at<float>(2,3) = -0.5f;
// kernelH.at<float>(2,4) = -0.5f;
// cv::filter2D(disparity->image, disparity->image, -1, kernelH);

}  // depth_map_fusion
