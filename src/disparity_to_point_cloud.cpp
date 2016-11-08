#include "disparity_to_point_cloud/disparity_to_point_cloud.hpp"

namespace d2pc {

void Disparity2PCloud::DisparityCb(const sensor_msgs::ImageConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Size s = disparity->image.size();
  cv::Vec3f *pv;

  cv::Mat median_filtered;
  // cv::Mat median_filtered = disparity->image;
  cv::medianBlur(disparity->image, median_filtered, 11);

  cv::Mat real_disparity;
  median_filtered.convertTo(real_disparity, CV_32FC1, 1.0 / 8.0);

  cv::Mat image3D;
  cv::reprojectImageTo3D(real_disparity, image3D, Q_);

  // cv::Mat medianFilterd = disparity->image;

  // cv::imshow("disparity", disparity->image);
  // cv::imshow("disparity median filter", medianFilterd);
  // cv::waitKey(0);  // Wait for a keystroke in the window
  // form 3 to width -3 : to remove outlire on the border
  for (int v = 30; v < s.height - 30; v++) {
    pv = image3D.ptr<cv::Vec3f>(v);
    for (int u = 30; u < s.width - 30; u++) {
      cv::Vec3f value = pv[u];
      cloud->points.push_back(pcl::PointXYZ(value[0], value[1], value[2]));
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  // send point cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  // does not work on the rosbag with the tf broadcaster for the camera position
  output.header.stamp = disparity->header.stamp;
  // TODO: create ros param for this
  output.header.frame_id = "/camera_optical_frame";
  p_cloud_pub_.publish(output);
  cloud.reset();
}

} /* d2pc */
