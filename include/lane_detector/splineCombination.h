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

          std::vector<cv::Point> right_spline;
          std::vector<cv::Point> left_spline;

          assert(spline1.size() > 3);
          assert(spline2.size() > 3);

          if(spline1[0].x > spline2[0].x)
          {
            right_spline = spline1;
            left_spline = spline2;
          }
          else
          {
            right_spline = spline2;
            left_spline = spline1;
          }

          int min_s1 = 9999;
          int max_s1 = -1;
          for(cv::Point p : spline1) {
            if(p.x > max_s1) max_s1 = p.x;
            if(p.x < min_s1) min_s1 = p.x;
          }

          int min_s2 = 9999;
          int max_s2 = -1;
          for(cv::Point p : spline2) {
            if(p.x > max_s2) max_s2 = p.x;
            if(p.x < min_s2) min_s2 = p.x;
          }

          int min_x = std::min(min_s1, min_s2);
          int max_x = std::max(max_s1, max_s2);

          bool found = false;
          for(int i = 0; i < left_spline.size()-1; i++) {
            cv::Point p1 = left_spline[i];
            cv::Point p2 = left_spline[i+1];
            float tangent_slope = calcSlope(p1, p2);
            for(int j = max_x; j >= min_x; j--) {
              int normal_y = cvRound(p1.y - (j - p1.x)/tangent_slope);
              cv::Point normal_point(j, normal_y);
              auto iterator = std::find(right_spline.begin(), right_spline.end(), normal_point);
              if(iterator != right_spline.end()) {
                normal_vector.push_back(p1);
                normal_vector.push_back(normal_point);
                lane_width = cv::norm(p1 - normal_point);
                //std::cout << "Got it!" << normal_point.x << ", " << normal_point.y << std::endl;
                break;
              }
            }
            if(found) break;
            else correct = false;
            //else std::cout << "Not to get it" << std::endl;
          }
          if(lane_width == 0) correct = false;
        };
};

#endif /* SPLINECOMBINATION_H_ */
