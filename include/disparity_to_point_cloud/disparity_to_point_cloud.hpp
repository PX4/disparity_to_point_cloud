#ifndef __DISPARITY_TO_POINT_CLOUD_HPP__
#define __DISPARITY_TO_POINT_CLOUD_HPP__

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace d2pc {
class Disparity2PCloud {
 private:
  ros::NodeHandle nh_;
  ros::Publisher p_cloud_pub_;
  ros::Subscriber disparity_sub_;
  // TODO import this coefficeint with the calibration file or camera info topic
  float fx_ = 714.24;
  float fy_ = 713.5;
  float cx_ = 376;
  float cy_ = 240;
  float base_line_ = 0.043;
  int min_count_ = 20;
  int threshold_ = 1;
  cv::Mat Q_;

 public:
  Disparity2PCloud() : nh_("~") {
    printf("Constructor start\n");
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

    cv::Mat K = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
    cv::Mat distCoeff1 = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat t = (cv::Mat_<double>(3, 1) << -base_line_, 0, 0);
    Q_ = cv::Mat::eye(4, 4, CV_64FC1);

    cv::Mat R1 = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat R2 = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1);
    cv::Mat P2 = cv::Mat::eye(3, 4, CV_64FC1);
    printf("Defining matrices\n");
    cv::Size s;
    s.height = 480;
    s.width = 752;
    cv::stereoRectify(K, distCoeff1, K, distCoeff1, s, R, t, R1, R2, P1, P2, Q_);
    printf("stereoRectify\n");
  };
  // virtual ~Disparity2PCloud();
  void DisparityCb(const sensor_msgs::ImageConstPtr &msg);
};
} /* d2pc */

#endif /* end of include guard: __DISPARITY_TO_POINT_CLOUD_HPP__ */
