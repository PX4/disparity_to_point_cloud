#ifndef __SCORE_FUNCTIONS_HPP__
#define __SCORE_FUNCTIONS_HPP__

#include <disparity_to_point_cloud/common.h>

namespace depth_map_fusion {

// Returns the (depth,score) from the better pair
inline
DepthScore betterScore(DepthScore hor_best, DepthScore hor_sec,
                       DepthScore ver_best, DepthScore ver_sec) {

  if (hor_best.score <= ver_best.score) {
    // A small score is better
    return hor_best;
  }
  return ver_best;
}

inline
DepthScore alwaysVer(DepthScore hor_best, DepthScore hor_sec,
                     DepthScore ver_best, DepthScore ver_sec) {

  return ver_best;
}

// Same as betterScore, uses the second-best from the other pair to get a new score 
inline
DepthScore secondBestInv(DepthScore hor_best, DepthScore hor_sec,
                         DepthScore ver_best, DepthScore ver_sec) {

  int hor_score = 100 - (ver_sec.score - hor_best.score);
  int ver_score = 100 - (hor_sec.score - ver_best.score);
  if (hor_score < ver_score) {
    // A small score is better
    return {hor_best.depth, hor_score};
  }
  return {ver_best.depth, ver_score};
}

// Same as secondBestInv, but with some slight hack 
inline
DepthScore secondBestInv2(DepthScore hor_best, DepthScore hor_sec,
                          DepthScore ver_best, DepthScore ver_sec) {
  int thres = 180;
  int tooClose = 230;
  int hor_score = 100 - (ver_sec.score - hor_best.score);
  int ver_score = 100 - (hor_sec.score - ver_best.score);

  float relative_diff = float(hor_best.depth) / float(ver_best.depth);

  if (hor_score < ver_score && hor_score < thres && hor_best.depth < tooClose && hor_best.depth!=0 && ver_best.depth!=0) {
    return hor_best;
  } 
  else if (ver_score < hor_score && ver_score < thres && ver_best.depth < tooClose && hor_best.depth!=0 && ver_best.depth!=0) {
    return ver_best;
  } 
  else if (0.8 < relative_diff && relative_diff < 1.25 && hor_score < 1.25 * thres 
          && ver_score < 1.25 * thres && hor_best.depth!=0 && ver_best.depth!=0) {
    
    return {(hor_best.depth + ver_best.depth) / 2, (hor_best.score + ver_best.score) / 2};
  }
  return {0, hor_best.score};
}

} // depth_map_fusion

#endif  // __SCORE_FUNCTIONS_HPP__
