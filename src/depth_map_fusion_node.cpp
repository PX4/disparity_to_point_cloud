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
 * @file depth_map_fusion_node.cpp
 *
 * Node which start the fusion class
 *
 * @author Vilhjálmur Vilhjálmsson <villi@px4.io>
 */

#include <boost/bind.hpp>

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <disparity_to_point_cloud/common.h>
#include "disparity_to_point_cloud/depth_map_fusion.hpp"
#include "disparity_to_point_cloud/camera_triplet.hpp"
#include "disparity_to_point_cloud/DepthMapFusionConfig.h"

namespace depth_map_fusion {
namespace dynamic_reconfiguration {

  // Evil globals are defined and modified here to enable easy dynamic reconfiguration
  bool LINE_DETECTION;
  int DEBUG_LEVEL;
  int THRESHOLD;
  int TOO_CLOSE;
  int FINAL_BLUR;
  int SCORE_MULT;
  int LINE_DET_WEIGHT;
  int SUB_MAT_MAX;
  int X_OFFSET;
  int Y_OFFSET;

  void callback(DepthMapFusionConfig & config, uint32_t level) {
    LINE_DETECTION  = config.line_detection;
    DEBUG_LEVEL     = config.debug_level;
    THRESHOLD       = config.threshold;
    TOO_CLOSE       = config.too_close;
    FINAL_BLUR      = config.final_blur;
    SCORE_MULT      = config.score_multiplier;
    LINE_DET_WEIGHT = config.line_detection_weight;
    SUB_MAT_MAX     = config.sub_mat_max;
    X_OFFSET        = config.x_offset;
    Y_OFFSET        = config.y_offset;
  }

} // dynamic_reconfiguration
} // depth_map_fusion


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "depth_map_fusion");
  ros::NodeHandle nh("~");

  // Set up Dynamic Reconfigure Server
  dynamic_reconfigure::Server<depth_map_fusion::DepthMapFusionConfig> server;
  auto f = boost::bind(&depth_map_fusion::dynamic_reconfiguration::callback, _1, _2);
  server.setCallback(f);

  // Get other parameters
  int base_id;
  bool hor_line_detection, ver_line_detection;
  bool disable_vertical = true;

  nh.param<int>("base_id", base_id, 0);
  nh.param<bool>("hor_line_detection", hor_line_detection, true);
  nh.param<bool>("ver_line_detection", ver_line_detection, true);

  // TODO: why does this not work?
  // nh.param<bool>("disable_vertical", disable_vertical, true); 

  // Create a Camera Triplet
  depth_map_fusion::CameraTriplet triplet(nh,
                                          base_id,
                                          hor_line_detection,
                                          ver_line_detection,
                                          disable_vertical);

  // Endless loop
  ros::spin();
  return 0;
}
