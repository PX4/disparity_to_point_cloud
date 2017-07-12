#ifndef __SCORE_FUNCTIONS_HPP__
#define __SCORE_FUNCTIONS_HPP__

#include <disparity_to_point_cloud/common.h>
#include <assert.h>

namespace depth_map_fusion {

namespace dynamic_reconfiguration {
  extern int THRESHOLD; // Evil global set in depth_map_fusion_node
  extern int TOO_CLOSE; // Evil global set in depth_map_fusion_node
}

// Returns the better pair
inline
DepthScore betterScore(DepthScore hor_best, DepthScore hor_sec,
                       DepthScore ver_best, DepthScore ver_sec) {

  if (hor_best.score <= ver_best.score) {
    // A small score is better
    return hor_best;
  }
  return ver_best;
}

// Returns the pair with the lower depth
inline
DepthScore lowerDepth(DepthScore hor_best, DepthScore hor_sec,
                      DepthScore ver_best, DepthScore ver_sec) {

  if (hor_best.depth >= ver_best.depth) {
    return hor_best;
  }
  return ver_best;
}

// Returns the vertical pair
inline
DepthScore alwaysVer(DepthScore hor_best, DepthScore hor_sec,
                     DepthScore ver_best, DepthScore ver_sec) {

  return ver_best;
}

// Same as betterScore, but uses the second-best score as well
inline
DepthScore secondBest(DepthScore hor_best, DepthScore hor_sec,
                      DepthScore ver_best, DepthScore ver_sec) {

  int hor_score = 100 - (hor_sec.score - hor_best.score);
  int ver_score = 100 - (ver_sec.score - ver_best.score);
  if (hor_score < ver_score) {
    return {hor_best.depth, hor_score};
  }
  return {ver_best.depth, ver_score};
}

// Same as secondBestInv, but with some slight hack 
inline
DepthScore secondBest2(DepthScore hor_best, DepthScore hor_sec,
                       DepthScore ver_best, DepthScore ver_sec) {
  int thres = 60;
  int tooClose = 230;
  int hor_score = 100 - (hor_sec.score - hor_best.score);
  int ver_score = 100 - (ver_sec.score - ver_best.score);

  assert(hor_score >= 0 && ver_score >= 0);

  float relative_diff = float(hor_best.depth) / float(ver_best.depth);

  if (hor_score < ver_score && hor_score < thres && hor_best.depth < tooClose && hor_best.depth!=0) {
    return hor_best;
  } 
  else if (ver_score < hor_score && ver_score < thres && ver_best.depth < tooClose && ver_best.depth!=0) {
    return ver_best;
  } 
  else if (0.67 < relative_diff && relative_diff < 1.5) {
    
    return {(hor_best.depth + ver_best.depth) / 2, (hor_best.score + ver_best.score) / 2};
  }
  return {0, hor_best.score};
}

// Returns an average of the two pairs if they roughly agree on the depth
inline
DepthScore onlyGoodOnes(DepthScore hor_best, DepthScore hor_sec,
                        DepthScore ver_best, DepthScore ver_sec) {

  int thres = 180;
  int tooClose = 230;
  int hor_score = hor_best.score;
  int ver_score = ver_best.score;

  float relative_diff = float(hor_best.depth) / float(ver_best.depth);

  // if (hor_score < ver_score && hor_score < thres && hor_best.depth < tooClose && hor_best.depth!=0 && ver_best.depth!=0) {
  //   return hor_best;
  // } 
  // else if (ver_score < hor_score && ver_score < thres && ver_best.depth < tooClose && hor_best.depth!=0 && ver_best.depth!=0) {
  //   return ver_best;
  // } 
  if (0.5 < relative_diff && relative_diff < 2.0) {
    return {(hor_best.depth + ver_best.depth) / 2, (hor_best.score + ver_best.score) / 2};
  }
  return {0, hor_best.score};
}

inline
DepthScore sobelFusion(int hor_depth, int hor_score,
                       int ver_depth, int ver_score) {
  int thres = dynamic_reconfiguration::THRESHOLD;
  int tooClose = dynamic_reconfiguration::TOO_CLOSE;

  assert(hor_score >= 0 && ver_score >= 0);

  float relative_diff = float(hor_depth) / float(ver_depth);

  if (hor_score < ver_score && hor_score < thres && hor_depth < tooClose && hor_depth!=0) {
    return {hor_depth, hor_score};
  } 
  else if (ver_score < hor_score && ver_score < thres && ver_depth < tooClose && ver_depth!=0) {
    return {ver_depth, ver_score};
  } 
  else if (0.75 < relative_diff && relative_diff < 1.33) {
    
    return {(hor_depth + ver_depth) / 2, (hor_score + ver_score) / 2};
  }
  return {0, hor_score};
}

inline
DepthScore filterHor(int best_depth, int hor_score,
                     int sec_depth,  int sec_score) {
  int thres = dynamic_reconfiguration::THRESHOLD;
  int tooClose = dynamic_reconfiguration::TOO_CLOSE;

  if (hor_score < thres && best_depth < tooClose && best_depth!=0) {
    return {best_depth, hor_score};
  }
  if (std::abs(best_depth - sec_depth) < 2){
    // The best and second-best dephts are almost the same,
    // real depth is probably on the border of near- and far-field
    return {best_depth, sec_score};
  }
  return {0, 100};
}

} // depth_map_fusion

#endif  // __SCORE_FUNCTIONS_HPP__
