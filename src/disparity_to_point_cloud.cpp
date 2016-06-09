#include "disparity_to_point_cloud/disparity_to_point_cloud.hpp"

namespace d2pc {

void Disparity2PCloud::DisparityCb(const sensor_msgs::ImageConstPtr &msg) {

  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Size s = disparity->image.size();
  float disp, X, Y, Z;
  uchar *pf;
  // form 3 to width -3 : to remove outlire on the border
  for (int y = 3; y < s.height - 3; y++) {
    pf = disparity->image.ptr<uchar>(y);
    for (int x = 3; x < s.width - 3; x++) {
      // a disparity of 1 pixel is really difficult to detect -> usally outlier
      if (pf[x] > 1) {
        disp = pf[x] / 8.0f;
        Z = fx_ * base_line_ / disp;
        X = (x - cx_) * Z / fx_;
        Y = (y - cy_) * Z / fy_;
        cloud_.points.push_back(pcl::PointXYZ(X, Y, Z));
      }
    }
  }
  cloud_.width = cloud_.points.size();
  cloud_.height = 1;
  cloud_.is_dense = false;
  // TODO incorporate pcl::StatisticalOutlierRemoval filter
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
  //     new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud(cloud_.makeShared());
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(1.0);
  // sor.filter(*cloud_filtered);
  // send point cloud
  pcl::PCLPointCloud2 dummy;
  pcl::toPCLPointCloud2(cloud_, dummy);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(dummy, output);
  // should be:
  // output.header = disparity->header;
  // does not work on the rosbag with the tf broadcaster for the camera position
  output.header.stamp = ros::Time::now();
  // output.header.frame_id = "/world";
  output.header.frame_id = "/camera_optical_frame";
  p_cloud_pub_.publish(output);
  // clear points and cloud
  cloud_.points.clear();
}

} /* d2pc */
