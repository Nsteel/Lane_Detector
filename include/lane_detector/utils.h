/*
 * utils.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <lane_detector/LaneDetector.hh>
#include <lane_detector/DetectorConfig.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <lane_detector/splineCombination.h>

namespace lane_detector{

  enum Driving
  {
    on_the_left = 1,
    on_the_right = 0
  };

    namespace utils {

    /*inline float extrapolateLineX(float y, line& l) {
          float x = (y*(l[1].x-l[0].x) - l[1].x*l[0].y + l[0].x*l[1].y)/(l[1].y - l[0].y);
          return x;
    }
    inline float extrapolateLineY(float x, line& l) {
          float y = (l[0].y*(l[1].x-x) + l[1].y*(x-l[0].x))/(l[1].x - l[0].x);
          return y;
    }*/
    inline float calcSlope(cv::Point& pt1, const cv::Point& pt2) {
          float dY = pt2.y - pt1.y;
          float dX = pt2.x - pt1.x;
          float slope = dY/dX;
          return slope;
    }

    inline void boxes2Rects(std::vector<LaneDetector::Box>& boxes, std::vector<cv::Rect>& rects) {
      rects.clear();
      for(LaneDetector::Box box : boxes) {
        cv::Rect bounding_rect = box.box;
        rects.push_back(bounding_rect);
      }
    }

    inline void getRectsCentroids(const std::vector<cv::Rect>& rects, std::vector<cv::Point2f>& centroids) {
      centroids.clear();
      for(cv::Rect rect : rects) {
        int centroid_x = cvRound((rect.x + (rect.x + rect.width-1))/2);
        int centroid_y = cvRound((rect.y + (rect.y + rect.height-1))/2);
        cv::Point2f centroid(centroid_x, centroid_y);
        centroids.push_back(centroid);
      }
    }

    inline bool sortPointsY (const cv::Point& p1, const cv::Point& p2) { return (p1.y > p2.y); }

    /**
     * This function draws a spline on an image
     *
     * \param inImage the input/output image
     * \param splinePoints vector of the spline points
     * \param color color of the spline
     */
    inline void drawSpline(cv::Mat inImage, const std::vector<cv::Point>& splinePoints, const uint32_t& thickness, const cv::Scalar& color) {
      const cv::Point *pts = (const cv::Point*) cv::Mat(splinePoints).data;
      int npts = cv::Mat(splinePoints).rows;

      cv::polylines(inImage, &pts,&npts, 1,
                  false, 			// draw open contour
                      color,// colour RGB ordering (here = green)
                  thickness 		        // line thickness
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
    lanesConf.lowerQuantile = dynConfig.lowerQuantile;
    lanesConf.localMaxima =dynConfig.localMaxima;
    lanesConf.binarize = dynConfig.binarize;
    lanesConf.detectionThreshold = dynConfig.detectionThreshold;
    lanesConf.smoothScores = dynConfig.smoothScores;

    lanesConf.ipmVpPortion = dynConfig.ipmVpPortion;
    lanesConf.getEndPoints = dynConfig.getEndPoints;
    lanesConf.group = dynConfig.group;
    lanesConf.groupThreshold = dynConfig.groupThreshold;

    lanesConf.ransacLineNumSamples = dynConfig.ransacLineNumSamples;
    lanesConf.ransacLineNumIterations = dynConfig.ransacLineNumIterations;
    lanesConf.ransacLineNumGoodFit = dynConfig.ransacLineNumGoodFit;
    lanesConf.ransacLineThreshold = dynConfig.ransacLineThreshold;
    lanesConf.ransacLineScoreThreshold = dynConfig.ransacLineScoreThreshold;
    lanesConf.ransacLineBinarize = dynConfig.ransacLineBinarize;
    lanesConf.ransacLineWindow = dynConfig.ransacLineWindow;


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

    lanesConf.numStrips = dynConfig.numStrips;

    lanesConf.useGroundPlane = dynConfig.useGroundPlane;

    lanesConf.ipmWindowClear = dynConfig.ipmWindowClear;
    lanesConf.ipmWindowLeft = dynConfig.ipmWindowLeft;
    lanesConf.ipmWindowRight = dynConfig.ipmWindowRight;
    LaneDetector::DEBUG_LINES = dynConfig.debug_lines? 1 : 0;
    }

    /** This function sets the matrix to zero except for the mask window passed in
    *
    * \param inMat input/output matrix
    * \param mask the rectangle defining the mask: (xleft, ytop, width, height)
    */
    inline void setMat(cv::Mat& inMat, cv::Rect mask) {
     cv::Mat mask_mat = cv::Mat::zeros(inMat.size(), CV_8UC1);

     //clipping the mask
     if(mask.y < 0) mask.y = 0;
     else if(mask.y >= inMat.rows) mask.y = inMat.rows-1;
     if(mask.x < 0) mask.x = 0;
     else if(mask.x >= inMat.cols) mask.x = inMat.cols-1;

     //std::cout << 1 << std::endl;
     for(int i = mask.y; i < mask.y + mask.height; i++) {
       //std::cout << 2 << std::endl;
       uchar* pixel = mask_mat.ptr<uchar>(i);
       //std::cout << 3 << std::endl;
       for(int j = mask.x; j < mask.x + mask.width; j++) {
         //std::cout << 4 << std::endl;
         pixel[j] = 255;
         //std::cout << 5 << std::endl;
       }
     }
     cv::Mat aux;
     inMat.copyTo(aux, mask_mat);
     //std::cout << 6 << std::endl;
     inMat = aux;
     //std::cout << 7 << std::endl;
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

     template<class T>
     inline void combineVectorInPairs(const std::vector<T>& input, std::vector<std::vector<T>>& output) {
       output.clear();
       if(input.size() > 1) {
         for(uint32_t i = 0; i < input.size()-1; i++) {
           for(uint32_t j = i+1; j < input.size(); j++) {
             std::vector<T> combination {input[i], input[j]};
             output.push_back(combination);
           }
         }
       }
     }

     inline void makeSplineCombinations(const lane_detector::DetectorConfig& config, const LaneDetector::IPMInfo& ipmInfo, const std::vector<cv::Point2f>& centroids, const std::vector<std::vector<cv::Point>>& splines, std::vector<SplineCombination>& combinations) {
       combinations.clear();
       assert(centroids.size() == splines.size());
       if(centroids.size() > 1) {
         for(uint32_t i = 0; i < centroids.size()-1; i++) {
           for(uint32_t j = i+1; j < centroids.size(); j++) {
             if(splines[i].size() > 3 && splines[j].size() > 3) {
               SplineCombination combination(config, ipmInfo, splines[i], splines[j], i, j, centroids[i], centroids[j]);
               combinations.push_back(combination);
            }
           }
         }
       }
     }

    //Transform a vector of points of the ipm image to world coordinates (in meters)
    inline void ipmPoints2World(const std::vector<cv::Point2f>& input, std::vector<cv::Point2f>& output, const LaneDetector::IPMInfo& ipmInfo) {
      std::vector<cv::Point2f> out;
      for(cv::Point2f p : input) {
        CvPoint2D32f p_world = p;

        //x-direction
        p_world.x /= ipmInfo.xScale;
        p_world.x += ipmInfo.xLimits[0];
        //y-direction
        p_world.y /= ipmInfo.yScale;
        p_world.y = ipmInfo.yLimits[1] - p_world.y;

        p_world.x  /= 1000; //convert to meters
        p_world.y  /= 1000;
        out.push_back(p_world);
      }
      output = out;
    }

    //Converts a single CV-point (cv::Point2f) in ROS-point (geometry_msgs::Point32)
    // ROS conventions are being used
    geometry_msgs::Point32 cvtCvPointToROSPoint(const cv::Point2f& point) {
        geometry_msgs::Point32 point32;
        point32.x = point.y;
        point32.y = -point.x;
        point32.z = 0;

        return point32;
    }

    //Converts a vector of CV-points (cv::Point2f) in ROS-points (geometry_msgs::Point32)
    inline void cvtCvPoints2ROSPoints(const std::vector<cv::Point2f>& input, std::vector<geometry_msgs::Point32>& output) {
      output.clear();
      for(cv::Point2f p : input) {
        geometry_msgs::Point32 p_ros = cvtCvPointToROSPoint(p);
        output.push_back(p_ros);
      }
    }

    //Converts a vector of CV-points (cv::Point) in CV float points (cv::Point2f)
    inline std::vector<cv::Point2f> cvtCvPoint2CvPoint2f(const std::vector<cv::Point>& input) {
      std::vector<cv::Point2f> output;
      for(cv::Point p : input) {
        cv::Point2f p_float = p;
        output.push_back(p_float);
      }
      return output;
    }

  }
};

#endif /* UTILS_H_ */
