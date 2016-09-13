#include "disparity_to_point_cloud/depth_map_fusion.hpp"

namespace depth_map_fusion {

// Callbacks
void DepthMapFusion::DisparityCb1(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cropped_depth_1_ = cropToSquare(disparity->image, offset_x_, offset_y_);

  publishWithColor(msg, cropped_depth_1_, cropped_depth_1_pub_, -1);

  publishFusedDepthMap(msg);
}

void DepthMapFusion::DisparityCb2(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Mat rot_mat = rotateMat(disparity->image);
  cropped_depth_2_ = cropToSquare(rot_mat, -offset_x_, -offset_y_);

  publishWithColor(msg, cropped_depth_2_, cropped_depth_2_pub_, -1);
}

void DepthMapFusion::MatchingScoreCb1(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cropped_score_1_ = cropToSquare(disparity->image, offset_x_, offset_y_);
  
  // cropped_score_1_grad_ should eventually have high values around horizontal lines
  cv::GaussianBlur(cropped_score_1_, cropped_score_1_grad_, cv::Size(13,13), 3.0);
  cv::Sobel(cropped_score_1_grad_, cropped_score_1_grad_, -1, 0, 2, 7, 0.03);
  threshold(cropped_score_1_grad_, cropped_score_1_grad_, 30, 255, 0);
  cv::GaussianBlur(cropped_score_1_grad_, cropped_score_1_grad_, cv::Size(21,21), 10.0);
  cropped_score_1_grad_ = cropped_score_1_ + 2 * cropped_score_1_grad_;
  cropped_score_1_ = cropped_score_1_grad_;

  publishWithColor(msg, cropped_score_1_, cropped_score_1_pub_, cv::COLORMAP_BONE);
}

void DepthMapFusion::MatchingScoreCb2(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  cv::Mat rot_mat = rotateMat(disparity->image);
  cropped_score_2_ = cropToSquare(rot_mat, -offset_x_, -offset_y_);
  
  // cropped_score_2_grad_ should eventually have high values around vertical lines
  cv::GaussianBlur(cropped_score_2_, cropped_score_2_grad_, cv::Size(13,13), 3.0);
  cv::Sobel(cropped_score_2_grad_, cropped_score_2_grad_, -1, 2, 0, 7, 0.03);
  threshold(cropped_score_2_grad_, cropped_score_2_grad_, 30, 255, 0);
  cv::GaussianBlur(cropped_score_2_grad_, cropped_score_2_grad_, cv::Size(21,21), 10.0);
  cropped_score_2_grad_ = cropped_score_2_ + 2 * cropped_score_2_grad_;
  cropped_score_2_ = cropped_score_2_grad_;

  publishWithColor(msg, cropped_score_2_, cropped_score_2_pub_, cv::COLORMAP_BONE);
}


// Fuses together the last depthmaps and publishes
// If messages don't arrive sequencially they may have different time stamps
void DepthMapFusion::publishFusedDepthMap(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
  disparity->image = cropToSquare(disparity->image);

  if (cropped_depth_1_.empty() || cropped_depth_2_.empty() || cropped_score_1_.empty() || cropped_score_2_.empty()) {
    return; // Have not recieved all depth maps and scores
  }

  // For each pixel, calculate a new distance
  for (int i=0; i < cropped_depth_2_.rows; ++i) {
    for (int j=0; j < cropped_depth_2_.cols; ++j) {
      disparity->image.at<unsigned char>(i, j) = getFusedDistance(i, j);
    }
  }

  // Just remove the borders, probably overkill
  disparity->image = cropMat(disparity->image, 20, 20, 20, 20);

  publishWithColor(msg, disparity->image, grad_pub_, -1);

  sensor_msgs::Image fused_image;
  disparity->toImageMsg(fused_image);
  fused_pub_.publish(fused_image);

  // Print out the error between the two score images
  // int curr_error = cv::norm(cropped_score_1_, cropped_depth_2_);
  // previous_errors_. push_front(curr_error);
  // if (previous_errors_.size() > 10) {
  //   previous_errors_.pop_back();
  //   double sum_error = std::accumulate(previous_errors_.begin(), previous_errors_.end(), 0);
  //   std::cout << sum_error / 10.0 << std::endl;
  // }
}

// Returns the fused depth for pixel (i,j)
int DepthMapFusion::getFusedDistance(int i, int j) {
  int dist1 = cropped_depth_1_.at<unsigned char>(i, j);
  int dist2 = cropped_depth_2_.at<unsigned char>(i, j);
  int score1 = cropped_score_1_.at<unsigned char>(i, j);
  int score2 = cropped_score_2_.at<unsigned char>(i, j);
  int grad1 = cropped_score_1_grad_.at<unsigned char>(i, j);
  int grad2 = cropped_score_2_grad_.at<unsigned char>(i, j);

  // Choose which fusion function to use
  return gradFilter(dist1, dist2, score1, score2, grad1, grad2);
}

int DepthMapFusion::weightedAverage(int dist1, int dist2, int score1, int score2) {
  int weight1 = std::max(0.01, 1.0 - (0.5 * score1));
  int weight2 = std::max(0.01, 1.0 - (0.5 * score2));
  return (weight1*dist1 + weight2*dist2) / (weight1 + weight2);
}

int DepthMapFusion::maxDist(int dist1, int dist2, int score1, int score2) {
  // Lower value means larger distance
  return std::min(dist1, dist2);
}

int DepthMapFusion::maxDistUnlessBlack(int dist1, int dist2, int score1, int score2) {
  if (dist1 == 0 || dist2 == 0) {
    return std::max(dist1, dist2);
  }
  return std::min(dist1, dist2);
}

int DepthMapFusion::betterScore(int dist1, int dist2, int score1, int score2) {
  if (score1 < score2) {
    // A small score is better
    return dist1;
  }
  return dist2;
}

int DepthMapFusion::onlyGood1(int dist1, int dist2, int score1, int score2) {
  if (score2 < 50) {
    // A small score is better
    return dist2;
  }
  return 0;
}

int DepthMapFusion::onlyGoodAvg(int dist1, int dist2, int score1, int score2) {
  if (score1 < 100 && score2 <  100) {
    return (dist1 + dist2) / 2;
  }
  return 0;
}

int DepthMapFusion::overlap(int dist1, int dist2, int score1, int score2) {
  if (score1 < score2 && score1 < 20) {
    // A small score is better
    return 150;
  }
  else if (score2 < score1 && score2 < 20) {
    return 255;
  }
  return 0;
}

int DepthMapFusion::blackToWhite(int dist1, int dist2, int score1, int score2) {
  return 255 - score1;
}

int DepthMapFusion::gradFilter(int dist1, int dist2, int score1, int score2, int grad1, int grad2) {
  int thres = 80; 
  int tooClose = 230;
  if (score1 < score2 && score1 < thres && dist1 < tooClose) {
    return dist1;
  }
  else if (score2 < score1 && score2 < thres && dist2 < tooClose) {
    return dist2;
  }
  return 0;
}

cv::Mat DepthMapFusion::cropMat(const cv::Mat &mat, int left, int right, int top, int bottom) {
  int num_cols = mat.cols - left - right;
  int num_rows = mat.rows - top - bottom;
  cv::Rect region = cv::Rect(left, top, num_cols, num_rows);
  return mat(region);
}

// Returns a new square matrix, which is the center of mat 
// Shrinks the larger dimension of mat until it is square
cv::Mat DepthMapFusion::cropToSquare(const cv::Mat &mat, int offset_x, int offset_y) {
  cv::Rect region;
  int num_cols = mat.cols - std::abs(offset_x);
  int num_rows = mat.rows - std::abs(offset_y);
  int n = std::min(mat.cols, mat.rows) - std::max(std::abs(offset_x), std::abs(offset_y_));

  if (num_cols < num_rows) {
    int start_col = std::max(0, offset_x);
    int start_row = std::max(0, offset_y + (num_rows - num_cols) / 2);
    region = cv::Rect(start_col, start_row, n, n);
  }
  else {
    int start_col = std::max(0, offset_x + (num_cols - num_rows) / 2);
    int start_row = std::max(0, offset_y );
    region = cv::Rect(start_col, start_row, n, n);
  }
  return mat(region);
}

// Rotate mat 90 deg clockwise
cv::Mat DepthMapFusion::rotateMat(const cv::Mat &mat) {
  cv::Mat rot_mat;
  cv::transpose(mat, rot_mat);  
  cv::flip(rot_mat, rot_mat,1);
  return rot_mat;
}

void DepthMapFusion::publishWithColor(const sensor_msgs::ImageConstPtr &msg, const cv::Mat &mat,
                                      const ros::Publisher &publisher, int colormap) {

  cv::Mat colored_mat;
  if (colormap == -1) {
    colorizeDepth(mat, colored_mat);
  }
  else {
    cv::applyColorMap(mat, colored_mat, colormap);
  }

  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header; 
  out_msg.encoding = "rgb8";
  out_msg.image = colored_mat; 

  publisher.publish(out_msg.toImageMsg()); 
}

// Colorizes gray scale from blue to red, whithout changing black pixels 
void DepthMapFusion::colorizeDepth(const cv::Mat& gray, cv::Mat& rgb) {
  double maxDisp= 255;
  float S=1.f;
  float V=1.f ;

  rgb.create( gray.size(), CV_8UC3 );
  rgb = cv::Scalar::all(0);

  if( maxDisp < 1 )
    return;

  for(int y=0; y < gray.rows; y++) {
    for(int x=0; x < gray.cols; x++) {
      unsigned char d = 40 + 0.8 * gray.at<unsigned char>(y,x);
      unsigned int H = 255 - ((unsigned char)maxDisp - d) * 280/ (unsigned char)maxDisp;    
      unsigned int hi = (H/60) % 6;

      float f = H/60.f - H/60;
      float p = V * (1 - S);
      float q = V * (1 - f * S);
      float t = V * (1 - (1 - f) * S);

      cv::Point3f res;

      if( hi == 0 ) //R = V,  G = t,  B = p
          res = cv::Point3f( p, t, V );
      if( hi == 1 ) // R = q, G = V,  B = p
          res = cv::Point3f( p, V, q );
      if( hi == 2 ) // R = p, G = V,  B = t
          res = cv::Point3f( t, V, p );
      if( hi == 3 ) // R = p, G = q,  B = V
          res = cv::Point3f( V, q, p );
      if( hi == 4 ) // R = t, G = p,  B = V
          res = cv::Point3f( V, p, t );
      if( hi == 5 ) // R = V, G = p,  B = q
          res = cv::Point3f( q, p, V );

      unsigned char b = (unsigned char)(std::max(0.f, std::min (res.x, 1.f)) * 255.f);
      unsigned char g = (unsigned char)(std::max(0.f, std::min (res.y, 1.f)) * 255.f);
      unsigned char r = (unsigned char)(std::max(0.f, std::min (res.z, 1.f)) * 255.f);

      rgb.at<cv::Point3_<unsigned char> >(y,x) = cv::Point3_<unsigned char>(b, g, r);     

      if (d == 40) {
        rgb.at<cv::Point3_<unsigned char> >(y,x) = cv::Point3_<unsigned char>(0, 0, 0);     
      }
    }
  }
}

// void erode() {
//   cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(*msg, "mono8");
//   cropped_score_1_ = cropToSquare(disparity->image, offset_x_, offset_y_);
//   cv::GaussianBlur(cropped_score_1_, cropped_score_1_grad_, cv::Size(5,5), 10.0);
//   threshold(cropped_score_1_grad_, cropped_score_1_grad_, 50, 255, 1);
//   cv::GaussianBlur(cropped_score_1_grad_, cropped_score_1_grad_, cv::Size(11,11), 10.0);
//   cv::Sobel(cropped_score_1_grad_, cropped_score_1_grad_, -1, 0, 2, 7, 0.03);
//   threshold(cropped_score_1_grad_, cropped_score_1_grad_, 50, 255, 0);


//   cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(40,1));
//   // Apply morphology operations
//   cv::erode(cropped_score_1_grad_, cropped_score_1_grad_, horizontalStructure, cv::Point(-1, -1));
//   cv::dilate(cropped_score_1_grad_, cropped_score_1_grad_, horizontalStructure, cv::Point(-1, -1));
//   cv::GaussianBlur(cropped_score_1_grad_, cropped_score_1_grad_, cv::Size(21,21), 10.0);
//   cropped_score_1_grad_ = cropped_score_1_ + cropped_score_1_grad_;
// }



  // cv::GaussianBlur(cropped_score_1_, cropped_score_1_grad_, cv::Size(5,5), 10.0);
  // threshold(cropped_score_1_grad_, cropped_score_1_grad_, 50, 255, 1);
  // cv::Mat horizontalStructure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,20));
  // // Apply morphology operations
  // cv::erode(cropped_score_1_grad_, cropped_score_1_grad_, horizontalStructure, cv::Point(-1, -1));



  // cv::Mat kernelH(5, 5, CV_32F, float(0));
  // kernelH.at<float>(0,2) = 0.5f;
  // kernelH.at<float>(1,2) = 0.5f;
  // kernelH.at<float>(3,2) = 0.5f;
  // kernelH.at<float>(4,2) = 0.5f;
  // kernelH.at<float>(2,0) = -0.5f;
  // kernelH.at<float>(2,1) = -0.5f;
  // kernelH.at<float>(2,2) = 1.0f;
  // kernelH.at<float>(2,3) = -0.5f;
  // kernelH.at<float>(2,4) = -0.5f;
  // cv::filter2D(disparity->image, disparity->image, -1, kernelH);

} // depth_map_fusion
