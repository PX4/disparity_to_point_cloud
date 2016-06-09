#ifndef __DISPARITY_TO_POINT_CLOUD_HPP__
#define __DISPARITY_TO_POINT_CLOUD_HPP__

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace d2pc {
class Disparity2PCloud {
 private:
  ros::NodeHandle nh_;
  ros::Publisher p_cloud_pub_;
  ros::Subscriber disparity_sub_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  // TODO import this coefficeint with the calibration file
  float fx_ = 784;
  float fy_ = 784;
  float cx_ = 319.882;
  float cy_ = 235.885;
  float base_line_ = 0.06;

 public:
  Disparity2PCloud() : nh_("~") {
    disparity_sub_ =
        nh_.subscribe("/disparity", 1, &Disparity2PCloud::DisparityCb, this);

    p_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1, this);
  };
  // virtual ~Disparity2PCloud();
  void DisparityCb(const sensor_msgs::ImageConstPtr &msg);
};
} /* d2pc */

#endif /* end of include guard: __DISPARITY_TO_POINT_CLOUD_HPP__ */
