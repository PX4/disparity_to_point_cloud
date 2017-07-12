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
 * Class that creates two CamerPair's and publishes a fused depthmap
 *
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#ifndef __CAMERA_TRIPLET_HPP__
#define __CAMERA_TRIPLET_HPP__

#include <disparity_to_point_cloud/camera_pair.hpp>
#include <disparity_to_point_cloud/score_functions.hpp>

namespace depth_map_fusion {

class CameraTriplet {
 public:
  CameraTriplet(ros::NodeHandle &nh,
                int id_,
                bool hor_line_detection=true,
                bool ver_line_detection=true,
                bool disable_vertical=true);

  // A pair received an image, if all images have that same timestamp, fuse them together
  void fuseIfPossible(const sensor_msgs::ImageConstPtr &msg);

  // Fuses the depthmaps from hor_pair and ver_pair, and publishes the result
  void fuseTriplet(const sensor_msgs::ImageConstPtr &msg);

  // Returns the fused depth and score for pixel (i,j)
  DepthScore getFusedPixel(int i, int j);

  int id;
  bool disable_vertical;
  CameraPair hor_pair;
  CameraPair ver_pair;
  ros::Publisher fused_depth_pub;
  ros::Publisher fused_depth_color_pub;
  ros::Publisher fused_depth_raw_pub;
  ros::Publisher fused_score_pub;
  ros::Publisher fused_score_raw_pub;
};

}  // depth_map_fusion

#endif  // __CAMERA_TRIPLET_HPP__
