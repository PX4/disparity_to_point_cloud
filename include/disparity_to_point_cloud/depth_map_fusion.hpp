#ifndef __DEPTH_MAP_FUSION_HPP__
#define __DEPTH_MAP_FUSION_HPP__

#include <deque>
#include <numeric>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace depth_map_fusion {

class DepthMapFusion {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber disparity1_sub_;
  ros::Subscriber disparity2_sub_;
  ros::Subscriber matching_score_1_sub_;
  ros::Subscriber matching_score_2_sub_;
  
  ros::Publisher fused_pub_;
  ros::Publisher cropped_depth_1_pub_;
  ros::Publisher cropped_depth_2_pub_;
  ros::Publisher cropped_score_1_pub_;
  ros::Publisher cropped_score_2_pub_;
  ros::Publisher grad_pub_;

  cv::Mat cropped_depth_1_;
  cv::Mat cropped_depth_2_;
  cv::Mat cropped_score_1_;
  cv::Mat cropped_score_2_;
  cv::Mat cropped_score_1_grad_;
  cv::Mat cropped_score_2_grad_;

  std::deque<int> previous_errors_;

 public:
  int offset_x_ = 0;
  int offset_y_ = 0;
  double scaling_factor_ = 1.0;

  DepthMapFusion() : nh_("~") {
    disparity1_sub_ = nh_.subscribe("/disparity_1", 1, &DepthMapFusion::DisparityCb1, this);
    disparity2_sub_ = nh_.subscribe("/disparity_2", 1, &DepthMapFusion::DisparityCb2, this);
    matching_score_1_sub_ = nh_.subscribe("/matching_score_1", 1, &DepthMapFusion::MatchingScoreCb1, this);
    matching_score_2_sub_ = nh_.subscribe("/matching_score_2", 1, &DepthMapFusion::MatchingScoreCb2, this);

    cropped_depth_1_pub_ = nh_.advertise<sensor_msgs::Image>("/cropped_depth_1", 5);
    cropped_depth_2_pub_ = nh_.advertise<sensor_msgs::Image>("/cropped_depth_2", 5);
    cropped_score_1_pub_ = nh_.advertise<sensor_msgs::Image>("/cropped_score_1", 5);
    cropped_score_2_pub_ = nh_.advertise<sensor_msgs::Image>("/cropped_score_2", 5);
    fused_pub_ = nh_.advertise<sensor_msgs::Image>("/fused_depth_map", 5);
    grad_pub_ = nh_.advertise<sensor_msgs::Image>("/gradient", 5);

    if (!nh_.getParam("offset_x", offset_x_)) {
      ROS_WARN("Failed to load parameter offset_x");
    }
    if (!nh_.getParam("offset_y", offset_y_)) {
      ROS_WARN("Failed to load parameter offset_y");
    }
  };
  
  void DisparityCb1(const sensor_msgs::ImageConstPtr &msg);
  void DisparityCb2(const sensor_msgs::ImageConstPtr &msg);
  void MatchingScoreCb1(const sensor_msgs::ImageConstPtr &msg);
  void MatchingScoreCb2(const sensor_msgs::ImageConstPtr &msg);

  void publishFusedDepthMap(const sensor_msgs::ImageConstPtr &msg);
  
  int getFusedDistance(int i, int j);
  int weightedAverage(int dist1, int dist2, int score1, int score2);
  int maxDist(int dist1, int dist2, int score1, int score2);
  int maxDistUnlessBlack(int dist1, int dist2, int score1, int score2);
  int betterScore(int dist1, int dist2, int score1, int score2);
  int onlyGood1(int dist1, int dist2, int score1, int score2);
  int onlyGoodAvg(int dist1, int dist2, int score1, int score2);
  int overlap(int dist1, int dist2, int score1, int score2);
  int blackToWhite(int dist1, int dist2, int score1, int score2);
  int gradFilter(int dist1, int dist2, int score1, int score2, int grad1, int grad2);
  
  cv::Mat cropMat(const cv::Mat &mat, int left, int right, int top, int bottom);
  cv::Mat cropToSquare(const cv::Mat &mat, int offset_x = 0, int offset_y = 0);
  cv::Mat rotateMat(const cv::Mat &mat);

  void publishWithColor(const sensor_msgs::ImageConstPtr &msg, 
                        const cv::Mat &mat,
                        const ros::Publisher &publisher,
                        int colormap);

  void colorizeDepth(const cv::Mat& gray, cv::Mat& rgb);

};

} // depth_map_fusion

#endif // __DEPTH_MAP_FUSION_HPP__
