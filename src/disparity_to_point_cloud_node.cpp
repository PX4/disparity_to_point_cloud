#include "disparity_to_point_cloud/disparity_to_point_cloud.hpp"

#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "disparity_to_point_cloud");
  d2pc::Disparity2PCloud d2pcloud;
  ros::spin();
  return 0;
}
