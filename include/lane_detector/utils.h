/*
 * utils.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <lane_detector/LaneDetector.hh>

namespace utils {
inline float extrapolateLineX(float y, line& l) {
        float x = (y*(l[1].x-l[0].x) - l[1].x*l[0].y + l[0].x*l[1].y)/(l[1].y - l[0].y);
        return x;
}
inline float extrapolateLineY(float x, line& l) {
        float y = (l[0].y*(l[1].x-x) + l[1].y*(x-l[0].x))/(l[1].x - l[0].x);
        return y;
}
inline float calcSlopeAngle(cv::Point& pt1, cv::Point& pt2) {
        float dY = std::abs(pt2.y - pt1.y);
        float dX = std::abs(pt2.x - pt1.x);
        float slope = dY/dX;
        return 180*std::atan(slope)/CV_PI;
}

inline bool compareLanes (LaneDetector::Line l1, LaneDetector::Line l2) { return (l1.score > l2.score); }

/** \brief This function resize the bounding boxes
 * \in order to avoid overlapping boxes
 * \param size the size of image containing the lines
 * \param boxes a vector of output resized bounding boxes
 * \param type the type of lines (LINE_HORIZONTAL or LINE_VERTICAL)
 */
inline void resizeBoxes(std::vector<LaneDetector::Box> &boxes, LaneDetector::LineType type)
{
  int overlap;

  for(auto i = boxes.begin(); i != boxes.end(); i++)
  {
    for(auto j = i+1; j != boxes.end(); j++)
    {
      switch(type)
      {
        case LaneDetector::LINE_VERTICAL:
          //get one with smallest x, and compute the x2 - x1
          overlap = i->box.x < j->box.x  ?
          (i->box.x + i->box.width - j->box.x) : (j->box.x + j->box.width - i->box.x);

          break;

        //TODO Useful when detecting stop lanes
        case LaneDetector::LINE_HORIZONTAL:
          //get one with smallest y, and compute the y2 - y1
          overlap = i->box.y < j->box.y  ?
          (i->box.y + i->box.height - j->box.y) : (j->box.y + j->box.height - i->box.y);

          break;

      } //switch
      if(overlap >= 0) {
        if(i->box.x < j->box.x) {
          if(i->line_slope_type == LaneDetector::SLOPE_INCREASING && overlap+1 < j->box.width) {
            j->box.x += overlap+1;
            j->box.width -= overlap-1;
            ROS_DEBUG("Avoiding overlap of lines with increasing slopes");
          }
          else if(j->line_slope_type == LaneDetector::SLOPE_DECREASING && overlap+1 < i->box.width) {
            i->box.width -= overlap-1;
            ROS_DEBUG("Avoiding overlap of lines with decreasing slopes");
          }
        }
        else if(i->box.x > j->box.x) {
          if(j->line_slope_type == LaneDetector::SLOPE_INCREASING && overlap+1 < i->box.width) {
            i->box.x += overlap-1;
            i->box.width -= overlap-1;
            ROS_DEBUG("Avoiding overlap of lines with increasing slopes");
          }
          else if(i->line_slope_type == LaneDetector::SLOPE_DECREASING && overlap+1 < j->box.width) {
            j->box.width -= overlap-1;
            ROS_DEBUG("Avoiding overlap of lines with decreasing slopes");
          }
        }
      }
    } //for j
  } // for i
}

};

#endif /* UTILS_H_ */
