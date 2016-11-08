#include "disparity_to_point_cloud/depth_map_fusion.hpp"

#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "depth_map_fusion");
  depth_map_fusion::DepthMapFusion depth_map_fusion;
  ros::spin();
  return 0;
}
