#include "disparity_to_point_cloud/disparity_to_point_cloud.hpp"

namespace d2pc {

void Disparity2PCloud::DisparityCb(const sensor_msgs::ImageConstPtr &msg) {
  printf("start \n");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  printf("new point cloud \n");
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  printf("toCvCopy \n");
  cv::Size s = disparity->image.size();
  cv::Vec3f *pv;

  cv::Mat median_filtered(s, CV_8U);
  // cv::Mat median_filtered = disparity->image;
  cv::medianBlur(disparity->image, median_filtered, 11);
  printf("medianBlur \n");

  cv::Mat real_disparity(s, CV_32FC1);
  median_filtered.convertTo(real_disparity, CV_32FC1, 1.0 / 8.0);

  cv::Mat image3D(s, CV_32FC3);
  cv::reprojectImageTo3D(real_disparity, image3D, Q_);
  printf("reprojectImageTo3D \n");

  // cv::Mat medianFilterd = disparity->image;

  // cv::imshow("disparity", disparity->image);
  // cv::imshow("disparity median filter", medianFilterd);
  // cv::waitKey(0);  // Wait for a keystroke in the window
  // form 3 to width -3 : to remove outlire on the border
  for (int v = 40; v < s.height - 40; v+=2) {
    pv = image3D.ptr<cv::Vec3f>(v);
    for (int u = 40; u < s.width - 40; u+=2) {
      cv::Vec3f value = pv[u];
      cloud->points.push_back(pcl::PointXYZ(value[0], value[1], value[2]));
    }
  }
  printf("cloud push_back \n");

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  printf("Cloud size: %d\n", cloud->points.size());
  // send point cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  // does not work on the rosbag with the tf broadcaster for the camera position
  output.header.stamp = disparity->header.stamp;
  // TODO: create ros param for this
  output.header.frame_id = "/camera_optical_frame";
  p_cloud_pub_.publish(output);
  printf("publish\n");
  // cloud.reset();
}

} /* d2pc */
