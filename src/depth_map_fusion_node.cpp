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

#include <disparity_to_point_cloud/common.h>
#include "disparity_to_point_cloud/depth_map_fusion.hpp"
#include "disparity_to_point_cloud/camera_triplet.hpp"

#include <ros/ros.h>

namespace depth_map_fusion {
	// Evil globals are set in this file to enable  
	// easy reconfiguration without recompilation
	bool DEBUG;
	int THRESHOLD;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "depth_map_fusion");
  // depth_map_fusion::DepthMapFusion depth_map_fusion;
  ros::NodeHandle nh("~");

  int base_id;
  bool hor_line_detection, ver_line_detection;
  nh.param<int>("base_id", base_id, 0);
  nh.param<int>("threshold", depth_map_fusion::THRESHOLD, 20);
  nh.param<bool>("debug", depth_map_fusion::DEBUG, false);
  nh.param<bool>("hor_line_detection", hor_line_detection, true);
  nh.param<bool>("ver_line_detection", ver_line_detection, true);

  depth_map_fusion::CameraTriplet triplet(nh,
                                          base_id,
                                          hor_line_detection,
                                          ver_line_detection);

  // Endless loop
  ros::spin();
  return 0;
}
