/*
 * splineCombination.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef SPLINECOMBINATION_H_
#define SPLINECOMBINATION_H_

#include <vector>
#include <cv.h>
#include <lane_detector/utils.h>
#include <iostream>
#include <algorithm>

inline float calcSlope(cv::Point& pt1, const cv::Point& pt2) {
      float dY = pt2.y - pt1.y;
      float dX = pt2.x - pt1.x;
      float slope = dY/dX;
      return slope;
}

class SplineCombination {
public:
        inline SplineCombination() { correct = false; };
        inline SplineCombination(const std::vector<cv::Point>& s1, const std::vector<cv::Point>& s2, const uint32_t& spline1_idx, const uint32_t& spline2_idx, const cv::Point2f& centroid_s1, const cv::Point2f& centroid_s2)
        : spline1(s1), spline2(s2), spline1_idx(spline1_idx), spline2_idx(spline2_idx), centroid_spline1(centroid_s1), centroid_spline2(centroid_s2), lane_width(0)
        {
          calcLaneWidth();
          correct = true;
        };

        std::vector<cv::Point> spline1;
        std::vector<cv::Point> spline2;
        uint32_t spline1_idx;
        uint32_t spline2_idx;
        cv::Point2f centroid_spline1;
        cv::Point2f centroid_spline2;
        std::vector<cv::Point> normal_vector;
        float lane_width;
        bool correct;

private:
        inline void calcLaneWidth() {

          assert(spline1.size() > 3);
          assert(spline2.size() > 3);

          std::vector<cv::Point> closest_spline;
          std::vector<cv::Point> second_closest_spline;

          if(spline1[0].y > spline2[0].y)  {
            closest_spline = spline1;
            second_closest_spline = spline2;
          }
          else {
            closest_spline = spline2;
            second_closest_spline = spline1;
          }

          for(int i = 0; i < closest_spline.size(); i++) {
            if(closest_spline[i].y <= second_closest_spline[0].y) {
              lane_width = std::abs(cvRound((closest_spline[i].x - second_closest_spline[0].x)));
              //std::cout << "Middle: " << middle << std::endl;
              break;
            }
          }

          if(lane_width == 0) correct = false;
        };
};

#endif /* SPLINECOMBINATION_H_ */
