#include "disparity_to_point_cloud/disparity_to_point_cloud.hpp"

namespace d2pc {

void Disparity2PCloud::DisparityCb(const sensor_msgs::ImageConstPtr &msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Size s = disparity->image.size();
  float disp, X, Y, Z;
  uchar *pf;
  uchar *pf_m1;
  uchar *pf_m2;
  uchar *pf_p1;
  uchar *pf_p2;
  // cv::Mat discreateDisparity;
  // disparity->image.convertTo(discreateDisparity, CV_32FC1);
  cv::Mat medianFilterd = disparity->image;
  // cv::medianBlur(disparity->image, medianFilterd, 11);
  // cv::imshow("disparity", disparity->image);
  // cv::imshow("disparity median filter", medianFilterd);
  // cv::waitKey(0);  // Wait for a keystroke in the window
  // form 3 to width -3 : to remove outlire on the border
  for (int v = 30; v < s.height - 30; v++) {
    pf = medianFilterd.ptr<uchar>(v);
    pf_m1 = medianFilterd.ptr<uchar>(v - 1);
    pf_p1 = medianFilterd.ptr<uchar>(v + 1);
    pf_m2 = medianFilterd.ptr<uchar>(v - 2);
    pf_p2 = medianFilterd.ptr<uchar>(v + 2);
    for (int u = 30; u < s.width - 30; u++) {
      uchar value = pf[u];
      int count = 0;
      // a disparity of 1 pixel is really difficult to detect -> usally
      // outlier
      // minimal disparity 5 pixel
      if (value > 5) {
        // look in a 5x5 patch if there is consensus on the dispartiy if yes
        // keep point otherwise not
        if (abs(pf_m2[u - 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m2[u] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m2[u + 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m2[u - 2] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m2[u + 2] - value) < threshold_) {
          count++;
        }
        ////////////////////////////////////////////////////////////////////////
        if (abs(pf_m1[u - 2] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m1[u - 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m1[u] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m1[u + 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_m1[u + 2] - value) < threshold_) {
          count++;
        }
        ////////////////////////////////////////////////////////////////////////
        if (abs(pf[u - 2] - value) < threshold_) {
          count++;
        }
        if (abs(pf[u - 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf[u + 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf[u + 2] - value) < threshold_) {
          count++;
        }
        ////////////////////////////////////////////////////////////////////////
        if (abs(pf_p1[u - 2] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p1[u - 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p1[u] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p1[u + 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p1[u + 2] - value) < threshold_) {
          count++;
        }
        ////////////////////////////////////////////////////////////////////////
        if (abs(pf_p2[u - 2] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p2[u - 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p2[u] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p2[u + 1] - value) < threshold_) {
          count++;
        }
        if (abs(pf_p2[u + 2] - value) < threshold_) {
          count++;
        }

        if (count > min_count_) {
          disp = value / 8.0f;
          Z = fx_ * base_line_ / disp;
          X = (u - cx_) * Z / fx_;
          Y = (v - cy_) * Z / fy_;
          cloud->points.push_back(pcl::PointXYZ(X, Y, Z));
        }
      }
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
}

} /* d2pc */
