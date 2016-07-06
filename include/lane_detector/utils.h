/*
 * utils.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <lane_detector/LaneDetector.hh>
#include <lane_detector/LaneDetectorOpt.h>
#include <lane_detector/DetectorConfig.h>

namespace utils {

/*inline float extrapolateLineX(float y, line& l) {
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
}*/

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

inline void translateConfiguration(lane_detector::DetectorConfig& dynConfig, LaneDetector::LaneDetectorConf& lanesConf) {

  //init the strucure
  lanesConf.ipmWidth = dynConfig.ipmWidth;
  lanesConf.ipmHeight = dynConfig.ipmHeight;
  lanesConf.ipmLeft = dynConfig.ipmLeft;
  lanesConf.ipmRight = dynConfig.ipmRight;
  lanesConf.ipmBottom = dynConfig.ipmBottom;
  lanesConf.ipmTop = dynConfig.ipmTop;
  lanesConf.ipmInterpolation = dynConfig.ipmInterpolation;

  lanesConf.lineWidth = dynConfig.lineWidth;
  lanesConf.lineHeight = dynConfig.lineHeight;
  lanesConf.kernelWidth = dynConfig.kernelWidth;
  lanesConf.kernelHeight = dynConfig.kernelHeight;
  lanesConf.lowerQuantile =
      dynConfig.lowerQuantile;
  lanesConf.localMaxima =
      dynConfig.localMaxima;
  lanesConf.groupingType = dynConfig.groupingType;
  lanesConf.binarize = dynConfig.binarize;
  lanesConf.detectionThreshold =
      dynConfig.detectionThreshold;
  lanesConf.smoothScores =
      dynConfig.smoothScores;
  lanesConf.rMin = dynConfig.rMin;
  lanesConf.rMax = dynConfig.rMax;
  lanesConf.rStep = dynConfig.rStep;
  lanesConf.thetaMin = dynConfig.thetaMin * CV_PI/180;
  lanesConf.thetaMax = dynConfig.thetaMax * CV_PI/180;
  lanesConf.thetaStep = dynConfig.thetaStep * CV_PI/180;
  lanesConf.ipmVpPortion = dynConfig.ipmVpPortion;
  lanesConf.getEndPoints = dynConfig.getEndPoints;
  lanesConf.group = dynConfig.group;
  lanesConf.groupThreshold = dynConfig.groupThreshold;
  lanesConf.ransac = dynConfig.ransac;

  lanesConf.ransacLineNumSamples = dynConfig.ransacLineNumSamples;
  lanesConf.ransacLineNumIterations = dynConfig.ransacLineNumIterations;
  lanesConf.ransacLineNumGoodFit = dynConfig.ransacLineNumGoodFit;
  lanesConf.ransacLineThreshold = dynConfig.ransacLineThreshold;
  lanesConf.ransacLineScoreThreshold = dynConfig.ransacLineScoreThreshold;
  lanesConf.ransacLineBinarize = dynConfig.ransacLineBinarize;
  lanesConf.ransacLineWindow = dynConfig.ransacLineWindow;

  lanesConf.ransacSplineNumSamples = dynConfig.ransacSplineNumSamples;
  lanesConf.ransacSplineNumIterations = dynConfig.ransacSplineNumIterations;
  lanesConf.ransacSplineNumGoodFit = dynConfig.ransacSplineNumGoodFit;
  lanesConf.ransacSplineThreshold = dynConfig.ransacSplineThreshold;
  lanesConf.ransacSplineScoreThreshold = dynConfig.ransacSplineScoreThreshold;
  lanesConf.ransacSplineBinarize = dynConfig.ransacSplineBinarize;
  lanesConf.ransacSplineWindow = dynConfig.ransacSplineWindow;

  lanesConf.ransacSplineDegree = dynConfig.ransacSplineDegree;

  lanesConf.ransacSpline = dynConfig.ransacSpline;
  lanesConf.ransacLine = dynConfig.ransacLine;
  lanesConf.ransacSplineStep = dynConfig.ransacSplineStep;

  lanesConf.overlapThreshold = dynConfig.overlapThreshold;

  lanesConf.localizeAngleThreshold = dynConfig.localizeAngleThreshold;
  lanesConf.localizeNumLinePixels = dynConfig.localizeNumLinePixels;

  lanesConf.extendAngleThreshold = dynConfig.extendAngleThreshold;
  lanesConf.extendMeanDirAngleThreshold = dynConfig.extendMeanDirAngleThreshold;
  lanesConf.extendLinePixelsTangent = dynConfig.extendLinePixelsTangent;
  lanesConf.extendLinePixelsNormal = dynConfig.extendLinePixelsNormal;
  lanesConf.extendContThreshold = dynConfig.extendContThreshold;
  lanesConf.extendDeviationThreshold = dynConfig.extendDeviationThreshold;
  lanesConf.extendRectTop = dynConfig.extendRectTop;
  lanesConf.extendRectBottom = dynConfig.extendRectBottom;

  lanesConf.extendIPMAngleThreshold = dynConfig.extendIPMAngleThreshold;
  lanesConf.extendIPMMeanDirAngleThreshold = dynConfig.extendIPMMeanDirAngleThreshold;
  lanesConf.extendIPMLinePixelsTangent = dynConfig.extendIPMLinePixelsTangent;
  lanesConf.extendIPMLinePixelsNormal = dynConfig.extendIPMLinePixelsNormal;
  lanesConf.extendIPMContThreshold = dynConfig.extendIPMContThreshold;
  lanesConf.extendIPMDeviationThreshold = dynConfig.extendIPMDeviationThreshold;
  lanesConf.extendIPMRectTop = dynConfig.extendIPMRectTop;
  lanesConf.extendIPMRectBottom = dynConfig.extendIPMRectBottom;

  lanesConf.splineScoreJitter = dynConfig.splineScoreJitter;
  lanesConf.splineScoreLengthRatio = dynConfig.splineScoreLengthRatio;
  lanesConf.splineScoreAngleRatio = dynConfig.splineScoreAngleRatio;
  lanesConf.splineScoreStep = dynConfig.splineScoreStep;

  lanesConf.splineTrackingNumAbsentFrames = dynConfig.splineTrackingNumAbsentFrames;
  lanesConf.splineTrackingNumSeenFrames = dynConfig.splineTrackingNumSeenFrames;

  lanesConf.mergeSplineThetaThreshold = dynConfig.mergeSplineThetaThreshold;
  lanesConf.mergeSplineRThreshold = dynConfig.mergeSplineRThreshold;
  lanesConf.mergeSplineMeanThetaThreshold = dynConfig.mergeSplineMeanThetaThreshold;
  lanesConf.mergeSplineMeanRThreshold = dynConfig.mergeSplineMeanRThreshold;
  lanesConf.mergeSplineCentroidThreshold = dynConfig.mergeSplineCentroidThreshold;

  lanesConf.lineTrackingNumAbsentFrames = dynConfig.lineTrackingNumAbsentFrames;
  lanesConf.lineTrackingNumSeenFrames = dynConfig.lineTrackingNumSeenFrames;

  lanesConf.mergeLineThetaThreshold = dynConfig.mergeLineThetaThreshold;
  lanesConf.mergeLineRThreshold = dynConfig.mergeLineRThreshold;

  lanesConf.numStrips = dynConfig.numStrips;


  lanesConf.checkSplines = dynConfig.checkSplines;
  lanesConf.checkSplinesCurvenessThreshold = dynConfig.checkSplinesCurvenessThreshold;
  lanesConf.checkSplinesLengthThreshold = dynConfig.checkSplinesLengthThreshold;
  lanesConf.checkSplinesThetaDiffThreshold = dynConfig.checkSplinesThetaDiffThreshold;
  lanesConf.checkSplinesThetaThreshold = dynConfig.checkSplinesThetaThreshold;

  lanesConf.checkIPMSplines = dynConfig.checkIPMSplines;
  lanesConf.checkIPMSplinesCurvenessThreshold = dynConfig.checkIPMSplinesCurvenessThreshold;
  lanesConf.checkIPMSplinesLengthThreshold = dynConfig.checkIPMSplinesLengthThreshold;
  lanesConf.checkIPMSplinesThetaDiffThreshold = dynConfig.checkIPMSplinesThetaDiffThreshold;
  lanesConf.checkIPMSplinesThetaThreshold = dynConfig.checkIPMSplinesThetaThreshold;

  lanesConf.finalSplineScoreThreshold = dynConfig.finalSplineScoreThreshold;

  lanesConf.useGroundPlane = dynConfig.useGroundPlane;

  lanesConf.checkColor = dynConfig.checkColor;
  lanesConf.checkColorNumBins = dynConfig.checkColorNumBins;
  lanesConf.checkColorWindow = dynConfig.checkColorWindow;
  lanesConf.checkColorNumYellowMin = dynConfig.checkColorNumYellowMin;
  lanesConf.checkColorRGMin = dynConfig.checkColorRGMin;
  lanesConf.checkColorRGMax = dynConfig.checkColorRGMax;
  lanesConf.checkColorGBMin = dynConfig.checkColorGBMin;
  lanesConf.checkColorRBMin = dynConfig.checkColorRBMin;
  lanesConf.checkColorRBFThreshold = dynConfig.checkColorRBFThreshold;
  lanesConf.checkColorRBF = dynConfig.checkColorRBF;

  lanesConf.ipmWindowClear = dynConfig.ipmWindowClear;
  lanesConf.ipmWindowLeft = dynConfig.ipmWindowLeft;
  lanesConf.ipmWindowRight = dynConfig.ipmWindowRight;

  lanesConf.checkLaneWidth = dynConfig.checkLaneWidth;
  lanesConf.checkLaneWidthMean = dynConfig.checkLaneWidthMean;
  lanesConf.checkLaneWidthStd = dynConfig.checkLaneWidthStd;
  LaneDetector::DEBUG_LINES = dynConfig.debug_lines? 1 : 0;
}

};

#endif /* UTILS_H_ */
