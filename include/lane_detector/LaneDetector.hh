/**
 * \file LaneDetector.hh
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date Thu 26 Jul, 2007
 *
 */

#ifndef LANEDETECTOR_HH_
#define LANEDETECTOR_HH_

#include <lane_detector/InversePerspectiveMapping.hh>
#include <lane_detector/mcv.hh>

namespace LaneDetector
{

//Debug global variable
extern int DEBUG_LINES;

///Line type
typedef enum LineType_ {
  LINE_HORIZONTAL = 0,
  LINE_VERTICAL = 1
} LineType;

///Slope type
typedef enum Slopetype_ {
  SLOPE_DECREASING = 0,
  SLOPE_INCREASING = 1
} SlopeType;

/// Line structure with start and end points
typedef struct Line
{
  ///start point
  FLOAT_POINT2D startPoint;
  ///end point
  FLOAT_POINT2D endPoint;
  ///Slope type of the line (SLOPE_INCREASING, SLOPE_DECREASING)
  SlopeType slope_type;
  ///score of line
  float score;
} Line;

/// Line structure with start and end points
typedef struct Box
{
  ///Rectangle of the box
  CvRect box;
  //Line inside the box
  Line line;
} Box;

///Structure to hold lane detector settings
typedef struct LaneDetectorConf
{
  ///width of IPM image to use
  FLOAT ipmWidth;
  ///height of IPM image
  FLOAT ipmHeight;
  ///Left point in original image of region to make IPM for
  int ipmLeft;
  ///Right point in original image of region to make IPM for
  int ipmRight;
  ///Top point in original image of region to make IPM for
  int ipmTop;
  ///Bottom point in original image of region to make IPM for
  int ipmBottom;
  ///The method to use for IPM interpolation
  int ipmInterpolation;
  ///width of line we are detecting
  FLOAT lineWidth;
  ///height of line we are detecting
  FLOAT lineHeight;
  ///kernel size to use for filtering
  unsigned char kernelWidth;
  unsigned char kernelHeight;
  ///lower quantile to use for thresholding the filtered image
  FLOAT lowerQuantile;
  ///whether to return local maxima or just the maximum
  bool localMaxima;
  ///whether to binarize the thresholded image or use the
  ///raw filtered image
  bool binarize;
  //unsigned char topClip;
  ///threshold for line scores to declare as line
  FLOAT detectionThreshold;
  ///whtehter to smooth the line scores detected or not
  bool smoothScores;
  ///portion of image height to add to y-coordinate of vanishing
  ///point when computing the IPM image
  float ipmVpPortion;
  ///get end points or not
  bool getEndPoints;
  ///group nearby lines
  bool group;
  ///threshold for grouping nearby lines
  float groupThreshold;
  ///RANSAC Line parameters
  int ransacLineNumSamples;
  int ransacLineNumIterations;
  int ransacLineNumGoodFit;
  float ransacLineThreshold;
  float ransacLineScoreThreshold;
  bool ransacLineBinarize;
  ///half width to use for ransac window
  int ransacLineWindow;

  ///Overlap threshold to use for grouping of bounding boxes
  float overlapThreshold;
  
  ///Whether to clear part of the IPM image
  bool ipmWindowClear;
  ///Left corrdinate of window to keep in IPM
  int ipmWindowLeft;
  ///Left corrdinate of window to keep in IPM
  int ipmWindowRight;

} LaneDetectorConf;

void mcvBinarizeImage(CvMat *inImage);
void mcvCumSum(const CvMat *inMat, CvMat *outMat);
void mcvDrawLine(CvMat *image, Line Line, CvScalar color, int width);
void mcvDrawRectangle(CvMat *image, CvRect rect, CvScalar color, int width);
void mcvDrawText(CvMat *image, char* str, CvPoint point, float size, CvScalar color);
void mcvFitRansacLine(const CvMat *image, int numSamples, int numIterations, float threshold, float scoreThreshold, int numGoodFit, bool getEndPoints, LineType lineType, Line *lineXY, float *lineRTheta, float *lineScore);
void mcvFitRobustLine(const CvMat *points, float *lineRTheta, float *lineAbc);
void mcvGet2DerivativeGaussianKernel(CvMat *kernel, unsigned char w, FLOAT sigma);
void mcvGetGaussianKernel(CvMat *kernel, unsigned char w, FLOAT sigma);
void mcvGetHVLines(const CvMat *inImage, vector <Line> *lines, vector <float> *lineScores, LineType lineType, FLOAT linePixelWidth, bool binarize, bool localMaxima, FLOAT detectionThreshold, bool smoothScores);
void mcvGetLinesBoundingBoxesVec(vector<Line> &lines, LineType type, CvSize size, vector<Box> &boxes);
void mcvGetLinesBoundingBoxes(const vector<Line> &lines, LineType type, CvSize size, vector<CvRect> &boxes);
void getLines(const CvMat* image, LineType lineType,
                 vector<Line> &lines, vector<float> &lineScores,
                 LaneDetectorConf *lineConf);
double mcvGetLocalMaxSubPixel(double val1, double val2, double val3);
CvMat* mcvGetNonZeroPoints(const CvMat *inMat, bool floatMat);
FLOAT mcvGetQuantile(const CvMat *mat, FLOAT qtile);
void mcvGetRansacLines(const CvMat *im, vector<Line> &lines, vector<float> &lineScores, LaneDetectorConf *lineConf, LineType lineType);
void mcvGetVectorMax(const CvMat *inVector, double *max, int *maxLoc, int ignore);
void mcvGroupBoundingBoxesVec(vector<Box> &boxes, LineType type, float groupThreshold);
void mcvGroupBoundingBoxes(vector<CvRect> &boxes, LineType type, float groupThreshold);
void mcvGroupLines(vector<Line> &lines, vector<float> &lineScores, float groupThreshold, CvSize bbox);
void mcvIntersectLineRThetaWithBB(FLOAT r, FLOAT theta, const CvSize bbox, Line *outLine);
void mcvIntersectLineWithBB(const Line *inLine, const CvSize bbox, Line *outLine);
bool mcvIsPointInside(FLOAT_POINT2D point, CvSize bbox);
void mcvLineXY2RTheta(const Line &line, float &r, float &theta);
void getIPM(CvMat **inImage, CameraInfo *cameraInfo, IPMInfo* ipmInfo, LaneDetectorConf *lanesConf, list<CvPoint>* outPixels);
void mcvSampleWeighted(const CvMat *cumSum, int numSamples, CvMat *randInd, CvRNG *rng);
void  mcvSetMat(CvMat *inMat, CvRect mask, double val);
void mcvThresholdLower(const CvMat *inMat, CvMat *outMat, FLOAT threshold);

}

#endif /*LANEDETECTOR_HH_*/
