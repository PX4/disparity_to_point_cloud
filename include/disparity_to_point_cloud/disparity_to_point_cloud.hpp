#ifndef __DISPARITY_TO_POINT_CLOUD_HPP__
#define __DISPARITY_TO_POINT_CLOUD_HPP__

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// filter point cloud
#include <pcl/filters/statistical_outlier_removal.h>
// plane detection
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace d2pc {
class Disparity2PCloud {
 private:
  ros::NodeHandle nh_;
  ros::Publisher p_cloud_pub_;
  ros::Subscriber disparity_sub_;

  // TODO import this coefficeint with the calibration file or camera info topic
  float fx_ = 784;
  float fy_ = 784;
  float cx_ = 319.882;
  float cy_ = 235.885;
  float base_line_ = 0.06;
  int min_count_ = 13;
  int threshold_ = 1;

 public:
  Disparity2PCloud() : nh_("~") {
    disparity_sub_ =
        nh_.subscribe("/disparity", 1, &Disparity2PCloud::DisparityCb, this);

    p_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1, this);
    if (!nh_.getParam("min_count", min_count_)) {
      ROS_WARN("Failed to load parameter min_count");
    }
    if (!nh_.getParam("disparity_threshold", threshold_)) {
      ROS_WARN("Failed to load parameter disparity_threshold");
    }
  };
  // virtual ~Disparity2PCloud();
  void DisparityCb(const sensor_msgs::ImageConstPtr &msg);
};
} /* d2pc */

#endif /* end of include guard: __DISPARITY_TO_POINT_CLOUD_HPP__ */
