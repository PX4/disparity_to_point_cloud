#ifndef __COMMON_HPP__
#define __COMMON_HPP__

namespace depth_map_fusion {

const int RAINBOW_WITH_BLACK = -1;
const int GRAY_SCALE = -2;

struct DepthScore {
  int depth;
  int score;
};

// The last 3 of the disparity image encode the score of each pixel
// The function copys the last 3 bits of each value in depth to score,
// and sets them to zero in score.
// Also scales score to values in [0,100]
inline
int moveScoreFromDepth(cv::Mat &depth, cv::Mat &score) {
  // if (score.size() != depth.size()) {
  score = depth.clone();
  // }

  cv::bitwise_and(depth, 7, score);     // Now score contains the last 3 bits of depth  
  cv::bitwise_and(depth, 255-7, depth); // Set the last 3 bits of depth to zero
  score *= 14;                          // Scale to ca. 0-100
}


// Rotate mat 90 deg
inline
cv::Mat rotateMat(const cv::Mat &mat, bool clockwise=false) {
  cv::Mat rot_mat;
  cv::transpose(mat, rot_mat);
  int axis = clockwise ? 1 : 0;
  cv::flip(rot_mat, rot_mat, axis);
  return rot_mat;
}


inline
cv::Mat cropMat(const cv::Mat &mat, int left, int right, int top, int bottom) {
  int num_cols = mat.cols - left - right;
  int num_rows = mat.rows - top - bottom;
  cv::Rect region = cv::Rect(left, top, num_cols, num_rows);
  return mat(region);
}


// Returns a new square matrix, which is the center of mat
// Shrinks the larger dimension of mat until it is square
inline
cv::Mat cropToSquare(const cv::Mat &mat, int offset_x=0, int offset_y=0) {
  cv::Rect region;
  int num_cols = mat.cols - std::abs(offset_x);
  int num_rows = mat.rows - std::abs(offset_y);
  int n = std::min(mat.cols, mat.rows) -
          std::max(std::abs(offset_x), std::abs(offset_y));

  if (num_cols < num_rows) {
    int start_col = std::max(0, offset_x);
    int start_row = std::max(0, offset_y + (num_rows - num_cols) / 2);
    region = cv::Rect(start_col, start_row, n, n);
  } else {
    int start_col = std::max(0, offset_x + (num_cols - num_rows) / 2);
    int start_row = std::max(0, offset_y);
    region = cv::Rect(start_col, start_row, n, n);
  }
  return mat(region);
}


// Colorizes gray scale from blue to red, whithout changing black pixels
inline
void colorizeDepth(const cv::Mat &gray, cv::Mat &rgb) {
  double maxDisp = 255;
  float S = 1.f;
  float V = 1.f;

  rgb.create(gray.size(), CV_8UC3);
  rgb = cv::Scalar::all(0);

  if (maxDisp < 1) return;

  for (int y = 0; y < gray.rows; y++) {
    for (int x = 0; x < gray.cols; x++) {
      unsigned char d = 40 + 0.8 * gray.at<unsigned char>(y, x);
      unsigned int H =
          255 - ((unsigned char)maxDisp - d) * 280 / (unsigned char)maxDisp;
      unsigned int hi = (H / 60) % 6;

      float f = H / 60.f - H / 60;
      float p = V * (1 - S);
      float q = V * (1 - f * S);
      float t = V * (1 - (1 - f) * S);

      cv::Point3f res;

      if (hi == 0)  // R = V,  G = t,  B = p
        res = cv::Point3f(p, t, V);
      if (hi == 1)  // R = q, G = V,  B = p
        res = cv::Point3f(p, V, q);
      if (hi == 2)  // R = p, G = V,  B = t
        res = cv::Point3f(t, V, p);
      if (hi == 3)  // R = p, G = q,  B = V
        res = cv::Point3f(V, q, p);
      if (hi == 4)  // R = t, G = p,  B = V
        res = cv::Point3f(V, p, t);
      if (hi == 5)  // R = V, G = p,  B = q
        res = cv::Point3f(q, p, V);

      unsigned char b =
          (unsigned char)(std::max(0.f, std::min(res.x, 1.f)) * 255.f);
      unsigned char g =
          (unsigned char)(std::max(0.f, std::min(res.y, 1.f)) * 255.f);
      unsigned char r =
          (unsigned char)(std::max(0.f, std::min(res.z, 1.f)) * 255.f);

      rgb.at<cv::Point3_<unsigned char> >(y, x) =
          cv::Point3_<unsigned char>(b, g, r);

      if (d == 40) {
        rgb.at<cv::Point3_<unsigned char> >(y, x) =
            cv::Point3_<unsigned char>(0, 0, 0);
      }
    }
  }
}


// Publish mat as an image, with the header from msg, with publisher
inline
void publishWithColor(const sensor_msgs::ImageConstPtr &msg,
                      const cv::Mat &mat,
                      const ros::Publisher &publisher,
                      int colormap,
                      std::string encoding="mono8") {

  if (colormap == GRAY_SCALE) {
    // publish original mat
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = encoding;
    out_msg.image = mat;
    publisher.publish(out_msg.toImageMsg());
    return;
  }

  cv::Mat colored_mat;
  if (colormap == RAINBOW_WITH_BLACK) {
    colorizeDepth(mat, colored_mat);
  } else {
    cv::applyColorMap(mat, colored_mat, colormap);
  }

  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;
  out_msg.encoding = "rgb8";
  out_msg.image = colored_mat;

  publisher.publish(out_msg.toImageMsg());
}

} // depth_map_fusion

#endif  // __COMMON_HPP__
