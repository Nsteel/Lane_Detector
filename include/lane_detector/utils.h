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
#include <opencv2/highgui/highgui.hpp>

namespace lane_detector{

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

    inline bool sortPointsY (const cv::Point& p1, const cv::Point& p2) { return (p1.y < p2.y); }

    /**
     * This function draws a spline on an image
     *
     * \param inImage the input/output image
     * \param splinePoints vector of the spline points
     * \param color color of the spline
     */
    inline void drawSpline(cv::Mat inImage, const std::vector<cv::Point>& splinePoints, const cv::Scalar& color) {
      const cv::Point *pts = (const cv::Point*) cv::Mat(splinePoints).data;
      int npts = cv::Mat(splinePoints).rows;

      cv::polylines(inImage, &pts,&npts, 1,
                  false, 			// draw open contour
                      color,// colour RGB ordering (here = green)
                  2 		        // line thickness
                  );
    }

    inline bool compareBoxes (const LaneDetector::Box& b1, const LaneDetector::Box& b2) { return (b1.line.score > b2.line.score); }

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

        //TODO IMPROVE!!
        } //switch
        if(overlap >= 0) {
          if(i->box.x < j->box.x) {
            if(i->line.slope_type == LaneDetector::SLOPE_INCREASING && overlap+2 < j->box.width) {
              j->box.x += overlap+2;
              j->box.width -= overlap-2;
              ROS_DEBUG("Avoiding overlap of lines with increasing slopes");
            }
            else if(j->line.slope_type == LaneDetector::SLOPE_DECREASING && overlap+2 < i->box.width) {
              i->box.width -= overlap-2;
              ROS_DEBUG("Avoiding overlap of lines with decreasing slopes");
            }
          }
          else if(i->box.x > j->box.x) {
            if(j->line.slope_type == LaneDetector::SLOPE_INCREASING && overlap+2 < i->box.width) {
              i->box.x += overlap-2;
              i->box.width -= overlap-2;
              ROS_DEBUG("Avoiding overlap of lines with increasing slopes");
            }
            else if(i->line.slope_type == LaneDetector::SLOPE_DECREASING && overlap+2 < j->box.width) {
              j->box.width -= overlap-2;
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

    /** This function sets the matrix to zero except for the mask window passed in
    *
    * \param inMat input/output matrix
    * \param mask the rectangle defining the mask: (xleft, ytop, width, height)
    */
    inline void setMat(cv::Mat& inMat, cv::Rect mask) {
     cv::Mat mask_mat = cv::Mat::zeros(inMat.size(), CV_8UC1);
     for(int i = mask.y; i < mask.y + mask.height; i++) {
       uchar* pixel = mask_mat.ptr<uchar>(i);
       for(int j = mask.x; j < mask.x + mask.width; j++) {
         pixel[j] = 255;
       }
     }
     cv::Mat aux;
     inMat.copyTo(aux, mask_mat);
     inMat = aux;
    }

    /**
     * This function scales the input image to have values 0->255
     *
     * \param inImage the input image
     * \param outImage hte output iamge
     */
     inline void scaleMat(cv::Mat& inImage, cv::Mat& outImage) {
       double min, max;
       cv::minMaxIdx(inImage, &min, 0);
       cv::subtract(inImage, cv::Scalar(min), outImage);
       inImage.convertTo(inImage, CV_32FC1);
       cv::minMaxIdx(inImage, 0, &max);
       inImage = inImage * 1/max;
     }
   }
};

#endif /* UTILS_H_ */
