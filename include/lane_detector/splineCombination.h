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
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/DetectorConfig.h>

inline float calcSlope(cv::Point& pt1, const cv::Point& pt2) {
      float dY = pt2.y - pt1.y;
      float dX = pt2.x - pt1.x;
      float slope = dY/dX;
      return slope;
}

class SplineCombination {
public:
        inline SplineCombination() { correct = false; num_absent_frames = 0;};
        inline SplineCombination(const lane_detector::DetectorConfig& config, const LaneDetector::IPMInfo& ipmInfo, const std::vector<cv::Point>& s1, const std::vector<cv::Point>& s2, const uint32_t& spline1_idx, const uint32_t& spline2_idx, const cv::Point2f& centroid_s1, const cv::Point2f& centroid_s2)
        : config(config), ipmInfo(ipmInfo), spline1(s1), spline2(s2), spline1_idx(spline1_idx), spline2_idx(spline2_idx), centroid_spline1(centroid_s1), centroid_spline2(centroid_s2), lane_width(0), num_absent_frames(0)
        {
          calcLaneWidth();
        };

        inline float calcCost(SplineCombination& last_lane) {
          float desired_width = config.width_between_lines * ipmInfo.xScale;
          float width_diff = (desired_width - lane_width);
          uint32_t width_weight = config.lane_width_weight;
          if(std::abs(width_diff) > config.width_between_lines_threshold) correct = false;

          std::array<float, 5> diff;
          if(correct) {

            if(centroid_spline1.x > centroid_spline2.x) {
               centroid_right = centroid_spline1;
               centroid_left = centroid_spline2;
               right_spline = spline1;
               left_spline = spline2;
             }
             else {
               centroid_right = centroid_spline2;
               centroid_left = centroid_spline1;
               right_spline = spline2;
               left_spline = spline1;
             }

             uint32_t length_spline1 = spline1.front().y - spline1.back().y;
             uint32_t length_spline2 = spline2.front().y - spline2.back().y;

             if(length_spline1 > length_spline2) {
               longest_spline = spline1;
               second_longest_spline = spline2;
             }
             else {
               longest_spline = spline2;
               second_longest_spline = spline1;
             }

            if(config.driving_orientation == 1)  //Driving on the left
            {
              diff[0] = centroid_right.x;
            }
            else //Driving on the right
            {
              diff[0] = centroid_left.x - (config.ipmWidth - 1);
            }
            diff[1] = std::min(length_spline1, length_spline2);
            diff[1] -= config.lineHeight * ipmInfo.yScale;
            diff[2] = width_weight*(desired_width - lane_width);
            diff[3] = last_lane.correct? cv::norm(centroid_left - last_lane.centroid_left) : 0;
            diff[4] = last_lane.correct? cv::norm(centroid_right - last_lane.centroid_right) : 0;
            //std::cout << "left_diff: " << diff[3] << " right_diff: " << diff[4] << std::endl;
            if(diff[3] > config.line_dist_threshold) {
              diff[3] = 1000;
            }
            if(diff[4] > config.line_dist_threshold)  {
              diff[4] = 1000;
            }
            //std::cout << ipmInfo.yScale << std::endl;
            //std::cout << "x1: " << centroid_spline1.x << " x2: " << centroid_spline2.x << " width: " << lane_width <<" d0: "<< diff[0] << " d1: " << diff[1] << " d2: " << diff[2] << std::endl;

            float dist = 0;
            for (float i = 0; i < diff.size(); ++i)
            {
              dist += diff[i] * diff[i];
            }
            return sqrtf(dist);
          }
          else { //not correct
            return 10000;
          }
        };

        std::vector<cv::Point> spline1;
        std::vector<cv::Point> spline2;
        std::vector<cv::Point> second_longest_spline;
        uint32_t spline1_idx;
        uint32_t spline2_idx;
        cv::Point2f centroid_spline1;
        cv::Point2f centroid_spline2;
        float lane_width;
        uint32_t num_absent_frames;
        std::vector<cv::Point> left_spline;
        std::vector<cv::Point> right_spline;
        cv::Point centroid_left, centroid_right;
        std::vector<cv::Point> longest_spline;
        bool correct;

private:
        LaneDetector::IPMInfo ipmInfo;
        lane_detector::DetectorConfig config;

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
              break;
            }
          }

          if(lane_width == 0) correct = false;
          else correct = true;
        };
};

#endif /* SPLINECOMBINATION_H_ */
