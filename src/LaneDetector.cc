/**
 * \file LaneDetector.cc
 * \author Mohamed Aly <malaa@caltech.edu>
 * \date Thu 26 Jul, 2007
 *
 */

#include <lane_detector/LaneDetector.hh>
#include <lane_detector/ranker.h>

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <math.h>

using namespace std;

#include <cv.h>
#include <highgui.h>

namespace LaneDetector
{
  // used for debugging
  int DEBUG_LINES = 0;

/**
 * This function gets a 1-D gaussian filter with specified
 * std deviation and range
 *
 * \param kernel input mat to hold the kernel (2*w+1x1)
 *      column vector (already allocated)
 * \param w width of kernel is 2*w+1
 * \param sigma std deviation
 */
void mcvGetGaussianKernel(CvMat *kernel, unsigned char w, FLOAT sigma)
{
  //get variance
  sigma *= sigma;

  //get the kernel
  for (double i=-w; i<=w; i++)
      CV_MAT_ELEM(*kernel, FLOAT_MAT_ELEM_TYPE, int(i+w), 0) =
          (FLOAT_MAT_ELEM_TYPE) exp(-(.5/sigma)*(i*i));
}

 /**
  * This function gets a 1-D second derivative gaussian filter
  * with specified std deviation and range
  *
  * \param kernel input mat to hold the kernel (2*w+1x1)
  *      column vector (already allocated)
  * \param w width of kernel is 2*w+1
  * \param sigma std deviation
  */
 void mcvGet2DerivativeGaussianKernel(CvMat *kernel,
                                      unsigned char w, FLOAT sigma)
 {
   //get variance
   sigma *= sigma;

   //get the kernel
   for (double i=-w; i<=w; i++)
       CV_MAT_ELEM(*kernel, FLOAT_MAT_ELEM_TYPE, int(i+w), 0) =
           (FLOAT_MAT_ELEM_TYPE)
           (exp(-.5*i*i)/sigma - (i*i)*exp(-(.5/sigma)*i*i)/(sigma*sigma));
 }

 /** This function performs the IPM transformation, filter and applies thresholding to the image.
  *
  * \param image the input/output image
  * \param cameraInfo the camera parameters
  * \param ipmInfo output for parameters of the performed IPM transformation
  * \param vp vanishing point
  * \param lanesConf parameters for lane detection
  */
 void getIpmMap(CvMat *inImage,
                  CameraInfo *cameraInfo, IPMInfo* ipmInfo, FLOAT_POINT2D vp, LaneDetectorConf *lanesConf, list<CvPoint>* outPixels,  list<CvPoint>* inPixels, list<CvPoint> *ipm_out_of_area, CvMat* uvGrid)
 {
   //input size
   CvSize inSize = cvSize(inImage->width, inImage->height);

   //Get IPM
   CvSize ipmSize = cvSize((int)lanesConf->ipmWidth,
       (int)lanesConf->ipmHeight);
   CvMat* ipm;
   ipm = cvCreateMat(ipmSize.height, ipmSize.width, inImage->type);
   ipmInfo->vpPortion = lanesConf->ipmVpPortion;
   ipmInfo->ipmLeft = lanesConf->ipmLeft;
   ipmInfo->ipmRight = lanesConf->ipmRight;
   ipmInfo->ipmTop = lanesConf->ipmTop;
   ipmInfo->ipmBottom = lanesConf->ipmBottom;
   ipmInfo->ipmInterpolation = lanesConf->ipmInterpolation;
   mcvGetIpmMap(inImage, ipm, uvGrid, ipmInfo, cameraInfo, vp, outPixels, inPixels, ipm_out_of_area);
   cvReleaseMat(&ipm);
 }

 /** This function thresholds the image below a certain value to the threshold
  * so: outMat(i,j) = inMat(i,j) if inMat(i,j)>=threshold
  *                 = threshold otherwise
  *
  * \param inMat input matrix
  * \param outMat output matrix
  * \param threshold threshold value
  *
  */
 void mcvThresholdLower(const CvMat *inMat, CvMat *outMat, FLOAT threshold)
 {

 #define MCV_THRESHOLD_LOWER(type) \
      for (int i=0; i<inMat->height; i++) \
         for (int j=0; j<inMat->width; j++) \
             if ( CV_MAT_ELEM(*inMat, type, i, j)<threshold) \
                 CV_MAT_ELEM(*outMat, type, i, j)=(type) 0; /*check it, was: threshold*/\

   //check if to copy into outMat or not
   if (inMat != outMat)
     cvCopy(inMat, outMat);

   //check type
   if (CV_MAT_TYPE(inMat->type)==FLOAT_MAT_TYPE)
   {
     MCV_THRESHOLD_LOWER(FLOAT_MAT_ELEM_TYPE)
   }
   else if (CV_MAT_TYPE(inMat->type)==INT_MAT_TYPE)
   {
     MCV_THRESHOLD_LOWER(INT_MAT_ELEM_TYPE)
   }
   else
   {
     cerr << "Unsupported type in mcvGetVectorMax\n";
     exit(1);
   }
 }

 /** This function gets the qtile-th quantile of the input matrix
  *
  * \param mat input matrix
  * \param qtile required input quantile probability
  * \return the returned value
  *
  */
 FLOAT mcvGetQuantile(const CvMat *mat, FLOAT qtile)
 {
   //make it a row vector
   CvMat rowMat;
   cvReshape(mat, &rowMat, 0, 1);

   //get the quantile
   FLOAT qval;
   qval = quantile((FLOAT*) rowMat.data.ptr, rowMat.width, qtile);

   return qval;
 }

 /** This function extracts lines from the passed infiltered and thresholded
  * image
  *
  * \param image the input thresholded filtered image
  * \param lineType the line type to look for (LINE_VERTICAL or LINE_HORIZONTAL)
  * \param lines a vector of lines
  * \param lineScores the line scores
  * \param lineConf the conf structure
  *
  */
 void getLines(const CvMat* image, LineType lineType,
                  vector<Line> &lines, vector<float> &lineScores,
                  LaneDetectorConf *lineConf)
{


   mcvGetHVLines(image, &lines, &lineScores, lineType,
               6, //stopLinePixelHeight,
               lineConf->binarize, lineConf->localMaxima,
               lineConf->detectionThreshold,
               lineConf->smoothScores);



  mcvGetRansacLines(image, lines, lineScores, lineConf, lineType);
 }

 /** This function groups the input filtered image into
  * horizontal or vertical lines.
  *
  * \param inImage input image
  * \param lines returned detected lines (vector of points)
  * \param lineScores scores of the detected lines (vector of floats)
  * \param lineType type of lines to detect
  *      LINE_HORIZONTAL (default) or LINE_VERTICAL
  * \param linePixelWidth width (or height) of lines to detect
  * \param localMaxima whether to detect local maxima or just get
  *      the maximum
  * \param detectionThreshold threshold for detection
  * \param smoothScores whether to smooth scores detected or not
  */
 void mcvGetHVLines(const CvMat *inImage, vector <Line> *lines,
                    vector <FLOAT> *lineScores, LineType lineType,
                    FLOAT linePixelWidth, bool binarize, bool localMaxima,
                    FLOAT detectionThreshold, bool smoothScores)
 {
   CvMat * image = cvCloneMat(inImage);
   //binarize input image if to binarize
   if (binarize)
   {
     //mcvBinarizeImage(image);
     image = cvCreateMat(inImage->rows, inImage->cols, INT_MAT_TYPE);
     cvThreshold(inImage, image, 0, 1, CV_THRESH_BINARY); //0.05
   }

   //get sum of lines through horizontal or vertical
   //sumLines is a column vector
   CvMat sumLines, *sumLinesp;
   int maxLineLoc = 0;
   switch (lineType)
   {
     case LINE_HORIZONTAL:
       sumLinesp = cvCreateMat(image->height, 1, FLOAT_MAT_TYPE);
       cvReduce(image, sumLinesp, 1, CV_REDUCE_SUM); //_AVG
       cvReshape(sumLinesp, &sumLines, 0, 0);
       //max location for a detected line
       maxLineLoc = image->height-1;
       break;
     case LINE_VERTICAL:
       sumLinesp = cvCreateMat(1, image->width, FLOAT_MAT_TYPE);
       cvReduce(image, sumLinesp, 0, CV_REDUCE_SUM); //_AVG
       cvReshape(sumLinesp, &sumLines, 0, image->width);
       //max location for a detected line
       maxLineLoc = image->width-1;
       break;
   }
     //SHOW_MAT(&sumLines, "sumLines:");

     //smooth it

   float smoothp[] =	{
     0.000003726653172, 0.000040065297393, 0.000335462627903, 0.002187491118183,
     0.011108996538242, 0.043936933623407, 0.135335283236613, 0.324652467358350,
     0.606530659712633, 0.882496902584595, 1.000000000000000, 0.882496902584595,
     0.606530659712633, 0.324652467358350, 0.135335283236613, 0.043936933623407,
     0.011108996538242, 0.002187491118183, 0.000335462627903, 0.000040065297393,
     0.000003726653172};
   int smoothWidth = 21;
   CvMat smooth = cvMat(1, smoothWidth, CV_32FC1, smoothp);
   if (smoothScores)
     cvFilter2D(&sumLines, &sumLines, &smooth);
 //     SHOW_MAT(&sumLines, "sumLines:");



   //get the max and its location
   vector <int> sumLinesMaxLoc;
   vector <double> sumLinesMax;
   int maxLoc; double max;
   //TODO: put the ignore in conf
   #define MAX_IGNORE 0 //(int(smoothWidth/2.)+1)
   #define LOCAL_MAX_IGNORE (int(MAX_IGNORE/4))
   mcvGetVectorMax(&sumLines, &max, &maxLoc, MAX_IGNORE);

   //put the local maxima stuff here
   if (localMaxima)
   {
     //loop to get local maxima
     for(int i=1+LOCAL_MAX_IGNORE; i<sumLines.rows-1-LOCAL_MAX_IGNORE; i++)
     {
 	    //get that value
 	    FLOAT val = CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE, i, 0);
 	    //check if local maximum
 	    if( (val > CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE, i-1, 0))
         && (val > CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE, i+1, 0))
         //		&& (i != maxLoc)
         && (val >= detectionThreshold) )
 	    {
         //iterators for the two vectors
         vector<double>::iterator j;
         vector<int>::iterator k;
         //loop till we find the place to put it in descendingly
         for(j=sumLinesMax.begin(), k=sumLinesMaxLoc.begin();
             j != sumLinesMax.end()  && val<= *j; j++,k++);
         //add its index
         sumLinesMax.insert(j, val);
         sumLinesMaxLoc.insert(k, i);
 	    }
     }
   }

   //check if didnt find local maxima
   if(sumLinesMax.size()==0 && max>detectionThreshold)
   {
     //put maximum
     sumLinesMaxLoc.push_back(maxLoc);
     sumLinesMax.push_back(max);
   }

 //     //sort it descendingly
 //     sort(sumLinesMax.begin(), sumLinesMax.end(), greater<double>());
 //     //sort the indices
 //     for (int i=0; i<(int)sumLinesMax.size(); i++)
 // 	for (int j=i; j<(int)sumLinesMax.size(); j++)
 // 	    if(sumLinesMax[i] == CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE,
 // 					     sumLinesMaxLoc[j], 0))
 // 	    {
 // 		int k = sumLinesMaxLoc[j];
 // 		sumLinesMaxLoc[j] = sumLinesMaxLoc[i];
 // 		sumLinesMaxLoc[i] = k;
 // 	    }
 //     //sort(sumLinesMaxLoc.begin(), sumLinesMaxLoc.end(), greater<int>());

     //plot the line scores and the local maxima
     //if(DEBUG_LINES) {//#ifdef DEBUG_GET_STOP_LINES
 //     gnuplot_ctrl *h =  mcvPlotMat1D(NULL, &sumLines, "Line Scores");
 //     CvMat *y = mcvVector2Mat(sumLinesMax);
 //     CvMat *x =  mcvVector2Mat(sumLinesMaxLoc);
 //     mcvPlotMat2D(h, x, y);
 //     //gnuplot_plot_xy(h, (double*)&sumLinesMaxLoc,(double*)&sumLinesMax, sumLinesMax.size(),"");
 //     cin.get();
 //     gnuplot_close(h);
 //     cvReleaseMat(&x);
 //     cvReleaseMat(&y);
 //}//#endif
   //process the found maxima
   for (int i=0; i<(int)sumLinesMax.size(); i++)
   {
     //get subpixel accuracy
     double maxLocAcc = mcvGetLocalMaxSubPixel(
       CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE, MAX(sumLinesMaxLoc[i]-1,0), 0),
       CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE, sumLinesMaxLoc[i], 0),
       CV_MAT_ELEM(sumLines, FLOAT_MAT_ELEM_TYPE,
                   MIN(sumLinesMaxLoc[i]+1,maxLineLoc), 0) );
     maxLocAcc += sumLinesMaxLoc[i];
     maxLocAcc = MIN(MAX(0, maxLocAcc), maxLineLoc);


 	//TODO: get line extent

 	//put the extracted line
     Line line;
     switch (lineType)
     {
       case LINE_HORIZONTAL:
         line.startPoint.x = 0.5;
         line.startPoint.y = (FLOAT)maxLocAcc + .5;//sumLinesMaxLoc[i]+.5;
         line.endPoint.x = inImage->width-.5;
         line.endPoint.y = line.startPoint.y;
         break;
       case LINE_VERTICAL:
         line.startPoint.x = (FLOAT)maxLocAcc + .5;//sumLinesMaxLoc[i]+.5;
         line.startPoint.y = .5;
         line.endPoint.x = line.startPoint.x;
         line.endPoint.y = inImage->height-.5;
         break;
     }
     (*lines).push_back(line);
     if (lineScores)
         (*lineScores).push_back(sumLinesMax[i]);
   }//for

   if(DEBUG_LINES)
   {//#ifdef DEBUG_GET_STOP_LINES
     CvMat *im, *im2 = cvCloneMat(image);
     if (binarize)
       cvConvertScale(im2, im2, 255, 0);

     if (binarize)
 	    im = cvCreateMat(image->rows, image->cols, CV_8UC3);
     else
 	    im = cvCreateMat(image->rows, image->cols, CV_32FC3);
     mcvScaleMat(im2, im2);
     cvCvtColor(im2, im, CV_GRAY2RGB);
     for (unsigned int i=0; i<lines->size(); i++)
     {
       Line line = (*lines)[i];
       mcvIntersectLineWithBB(&line, cvSize(image->cols, image->rows), &line);
       if (binarize)
         mcvDrawLine(im, line, CV_RGB(255,0,0), 1);
       else
         mcvDrawLine(im, line, CV_RGB(1,0,0), 1);
     }

     char str[256];
     switch (lineType)
     {
       case LINE_HORIZONTAL:
         sprintf(str, "%s", "Horizontal Lines");
         break;
       case LINE_VERTICAL:
         sprintf(str, "%s", "Vertical Lines");
         break;
     }
     SHOW_IMAGE(im, str, 10);
     cvReleaseMat(&im);
     cvReleaseMat(&im2);
   }

   //clean
   cvReleaseMat(&sumLinesp);
   //cvReleaseMat(&smooth);
   sumLinesMax.clear();
   sumLinesMaxLoc.clear();
   cvReleaseMat(&image);
 }


 /** This function gets the maximum value in a vector (row or column)
  * and its location
  *
  * \param inVector the input vector
  * \param max the output max value
  * \param maxLoc the location (index) of the first max
  *
  */
 #define MCV_VECTOR_MAX(type)  \
     /*row vector*/ \
     if (inVector->height==1) \
     { \
         /*initial value*/ \
         tmax = (double) CV_MAT_ELEM(*inVector, type, 0, inVector->width-1); \
         tmaxLoc = inVector->width-1; \
         /*loop*/ \
         for (int i=inVector->width-1-ignore; i>=0+ignore; i--) \
         { \
             if (tmax<CV_MAT_ELEM(*inVector, type, 0, i)) \
             { \
                 tmax = CV_MAT_ELEM(*inVector, type, 0, i); \
                 tmaxLoc = i; \
             } \
         } \
     } \
     /*column vector */ \
     else \
     { \
         /*initial value*/ \
         tmax = (double) CV_MAT_ELEM(*inVector, type, inVector->height-1, 0); \
         tmaxLoc = inVector->height-1; \
         /*loop*/ \
         for (int i=inVector->height-1-ignore; i>=0+ignore; i--) \
         { \
             if (tmax<CV_MAT_ELEM(*inVector, type, i, 0)) \
             { \
                 tmax = (double) CV_MAT_ELEM(*inVector, type, i, 0); \
                 tmaxLoc = i; \
             } \
         } \
     } \

 void mcvGetVectorMax(const CvMat *inVector, double *max, int *maxLoc, int ignore)
 {
   double tmax;
   int tmaxLoc;

   if (CV_MAT_TYPE(inVector->type)==FLOAT_MAT_TYPE)
   {
     MCV_VECTOR_MAX(FLOAT_MAT_ELEM_TYPE)
   }
   else if (CV_MAT_TYPE(inVector->type)==INT_MAT_TYPE)
   {
     MCV_VECTOR_MAX(INT_MAT_ELEM_TYPE)
   }
   else
   {
     cerr << "Unsupported type in mcvGetVectorMax\n";
     exit(1);
   }

   //return
   if (max)
       *max = tmax;
   if (maxLoc)
       *maxLoc = tmaxLoc;
 }

 /** This fits a parabola to the entered data to get
  * the location of local maximum with sub-pixel accuracy
  *
  * \param val1 first value
  * \param val2 second value
  * \param val3 third value
  *
  * \return the computed location of the local maximum
  */
 double mcvGetLocalMaxSubPixel(double val1, double val2, double val3)
 {
   //build an array to hold the x-values
   double Xp[] = {1, -1, 1, 0, 0, 1, 1, 1, 1};
   CvMat X = cvMat(3, 3, CV_64FC1, Xp);

   //array to hold the y values
   double yp[] = {val1, val2, val3};
   CvMat y = cvMat(3, 1, CV_64FC1, yp);

   //solve to get the coefficients
   double Ap[3];
   CvMat A = cvMat(3, 1, CV_64FC1, Ap);
   cvSolve(&X, &y, &A, CV_SVD);

   //get the local max
   double max;
   max = -0.5 * Ap[1] / Ap[0];

   //return
   return max;
 }

 /** This function intersects the input line with the given bounding box
  *
  * \param inLine the input line
  * \param bbox the bounding box
  * \param outLine the output line
  *
  */
 void mcvIntersectLineWithBB(const Line *inLine, const CvSize bbox,
                             Line *outLine)
 {
   //put output
   outLine->startPoint.x = inLine->startPoint.x;
   outLine->startPoint.y = inLine->startPoint.y;
   outLine->endPoint.x = inLine->endPoint.x;
   outLine->endPoint.y = inLine->endPoint.y;

   //check which points are inside
   bool startInside, endInside;
   startInside = mcvIsPointInside(inLine->startPoint, bbox);
   endInside = mcvIsPointInside(inLine->endPoint, bbox);

   //now check
   if (!(startInside && endInside))
   {
     //difference
     FLOAT deltax, deltay;
     deltax = inLine->endPoint.x - inLine->startPoint.x;
     deltay = inLine->endPoint.y - inLine->startPoint.y;
     //hold parameters
     FLOAT t[4]={2,2,2,2};
     FLOAT xup, xdown, yleft, yright;

     //intersect with top and bottom borders: y=0 and y=bbox.height-1
     if (deltay==0) //horizontal line
     {
       xup = xdown = bbox.width+2;
     }
     else
     {
       t[0] = -inLine->startPoint.y/deltay;
       xup = inLine->startPoint.x + t[0]*deltax;
       t[1] = (bbox.height-inLine->startPoint.y)/deltay;
       xdown = inLine->startPoint.x + t[1]*deltax;
     }

     //intersect with left and right borders: x=0 and x=bbox.widht-1
     if (deltax==0) //horizontal line
     {
       yleft = yright = bbox.height+2;
     }
     else
     {
       t[2] = -inLine->startPoint.x/deltax;
       yleft = inLine->startPoint.y + t[2]*deltay;
       t[3] = (bbox.width-inLine->startPoint.x)/deltax;
       yright = inLine->startPoint.y + t[3]*deltay;
     }

     //points of intersection
     FLOAT_POINT2D pts[4] = {{xup, 0},{xdown,bbox.height},
       {0, yleft},{bbox.width, yright}};

     //now decide which stays and which goes
     int i;
     if (!startInside)
     {
       bool cont=true;
       for (i=0; i<4 && cont; i++)
       {
         if (t[i]>=0 && t[i]<=1 && mcvIsPointInside(pts[i],bbox) &&
           !(pts[i].x == outLine->endPoint.x &&
           pts[i].y == outLine->endPoint.y) )
         {
           outLine->startPoint.x = pts[i].x;
           outLine->startPoint.y = pts[i].y;
           t[i] = 2;
           cont = false;
         }
       }
 	    //check if not replaced
 	    if(cont)
 	    {
         //loop again removing restriction on endpoint this time
         for (i=0; i<4 && cont; i++)
         {
           if (t[i]>=0 && t[i]<=1 && mcvIsPointInside(pts[i],bbox))
           {
             outLine->startPoint.x = pts[i].x;
             outLine->startPoint.y = pts[i].y;
             t[i] = 2;
             cont = false;
           }
         }
       }
     }
     if (!endInside)
     {
       bool cont=true;
       for (i=0; i<4 && cont; i++)
       {
         if (t[i]>=0 && t[i]<=1 && mcvIsPointInside(pts[i],bbox) &&
           !(pts[i].x == outLine->startPoint.x &&
           pts[i].y == outLine->startPoint.y) )
         {
           outLine->endPoint.x = pts[i].x;
           outLine->endPoint.y = pts[i].y;
           t[i] = 2;
           cont = false;
         }
       }
       //check if not replaced
       if(cont)
       {
         //loop again removing restriction on endpoint this time
         for (i=0; i<4 && cont; i++)
         {
           if (t[i]>=0 && t[i]<=1 && mcvIsPointInside(pts[i],bbox))
           {
             outLine->endPoint.x = pts[i].x;
             outLine->endPoint.y = pts[i].y;
             t[i] = 2;
             cont = false;
           }
         }
       }
     }
   }
 }

 /** This function checks if the given point is inside the bounding box
  * specified
  *
  * \param inLine the input line
  * \param bbox the bounding box
  * \param outLine the output line
  *
  */
 bool mcvIsPointInside(FLOAT_POINT2D point, CvSize bbox)
 {
   return (point.x>=0 && point.x<=bbox.width
       && point.y>=0 && point.y<=bbox.height) ? true : false;
 }

 /** This function performs a RANSAC validation step on the detected lines
  *
  * \param image the input image
  * \param inLines vector of lines
  * \param outLines vector of grouped lines
  * \param groupThreshold the threshold used for grouping
  * \param bbox the bounding box to intersect with
  * \param lineType the line type to work on (horizontal or vertical)
  */
 void mcvGetRansacLines(const CvMat *im, vector<Line> &lines,
                        vector<float> &lineScores, LaneDetectorConf *lineConf,
                        LineType lineType)
 {
   //check if to binarize image
   CvMat *image = cvCloneMat(im);
   if (lineConf->ransacLineBinarize)
     mcvBinarizeImage(image);

   int width = image->width-1;
   int height = image->height-1;
   //try grouping the lines into regions
   //float groupThreshold = 15;
   mcvGroupLines(lines, lineScores, lineConf->groupThreshold,
                 cvSize(width, height));

   //group bounding boxes of lines
   float overlapThreshold = lineConf->overlapThreshold; //0.5; //.8;
   vector<CvRect> boxes;
   mcvGetLinesBoundingBoxes(lines, lineType, cvSize(width, height),
                            boxes);
   mcvGroupBoundingBoxes(boxes, lineType, overlapThreshold);
   //     mcvGroupLinesBoundingBoxes(lines, lineType, overlapThreshold,
   // 			       cvSize(width, height), boxes);

   //     //check if there're no lines, then check the whole image
   //     if (boxes.size()<1)
   // 	boxes.push_back(cvRect(0, 0, width-1, height-1));

   int window = lineConf->ransacLineWindow; //15;
   vector<Line> newLines;
   vector<float> newScores;
   for (int i=0; i<(int)boxes.size(); i++) //lines
   {
     // 	fprintf(stderr, "i=%d\n", i);
     //Line line = lines[i];
     CvRect mask, box;
     //get box
     box = boxes[i];
     switch (lineType)
     {
       case LINE_HORIZONTAL:
       {
         //get extent
         //int ystart = (int)fmax(fmin(line.startPoint.y, line.endPoint.y)-window, 0);
         //int yend = (int)fmin(fmax(line.startPoint.y, line.endPoint.y)+window, height-1);
         int ystart = (int)fmax(box.y - window, 0);
         int yend = (int)fmin(box.y + box.height + window, height-1);
         //get the mask
         mask = cvRect(0, ystart, width, yend-ystart+1);
       }
       break;

       case LINE_VERTICAL:
       {
         //get extent of window to search in
         //int xstart = (int)fmax(fmin(line.startPoint.x, line.endPoint.x)-window, 0);
         //int xend = (int)fmin(fmax(line.startPoint.x, line.endPoint.x)+window, width-1);
         int xstart = (int)fmax(box.x - window, 0);
         int xend = (int)fmin(box.x + box.width + window, width-1);
         //get the mask
         mask = cvRect(xstart, 0, xend-xstart+1, height);
       }
       break;
     }
     //get the subimage to work on
     CvMat *subimage = cvCloneMat(image);
     //clear all but the mask
     mcvSetMat(subimage, mask, 0);

     //get the RANSAC line in this part
     //int numSamples = 5, numIterations = 10, numGoodFit = 15;
     //float threshold = 0.5;
     float lineRTheta[2]={-1,0};
     float lineScore;
     Line line;
     mcvFitRansacLine(subimage, lineConf->ransacLineNumSamples,
                      lineConf->ransacLineNumIterations,
                      lineConf->ransacLineThreshold,
                      lineConf->ransacLineScoreThreshold,
                      lineConf->ransacLineNumGoodFit,
                      lineConf->getEndPoints, lineType,
                      &line, lineRTheta, &lineScore);

     //store the line if found and make sure it's not
     //near horizontal or vertical (depending on type)
     #warning "check this screening in ransacLines"
     if (lineRTheta[0]>=0)
     {
       bool put =true;
       switch(lineType)
       {
         case LINE_HORIZONTAL:
           //make sure it's not vertical
           if (fabs(lineRTheta[1]) < 30*CV_PI/180)
             put = false;
           break;

         case LINE_VERTICAL:
           //make sure it's not horizontal
           if((fabs(lineRTheta[1]) > 20*CV_PI/180))
             put = false;
           break;
       }
       if (put)
       {
         newLines.push_back(line);
         newScores.push_back(lineScore);
       }
     } // if

     //debug
     if(DEBUG_LINES) {//#ifdef DEBUG_GET_STOP_LINES

       //get string
       char str[256];
       switch (lineType)
       {
         case LINE_HORIZONTAL:
           sprintf(str, "Subimage Line H #%d", i);
           break;
         case LINE_VERTICAL:
           sprintf(str, "Subimage Line V #%d", i);
           break;
       }
       //convert image to rgb
       mcvScaleMat(subimage, subimage);
       CvMat *subimageClr = cvCreateMat(subimage->rows, subimage->cols,
                                        CV_32FC3);
       cvCvtColor(subimage, subimageClr, CV_GRAY2RGB);
       //draw rectangle
       //      	    mcvDrawRectangle(subimageClr, box,
                   // 			     CV_RGB(255, 255, 0), 1);
       mcvDrawRectangle(subimageClr, mask, CV_RGB(255, 255, 255), 1);

       //draw line
       if (lineRTheta[0]>0)
         mcvDrawLine(subimageClr, line, CV_RGB(1,0,0), 1);
       SHOW_IMAGE(subimageClr, str, 10);
       //clear
       cvReleaseMat(&subimageClr);
     }//#endif

     //clear
     cvReleaseMat(&subimage);
   } // for i

   //group lines
   vector<Line> oldLines;
   if (DEBUG_LINES)
     oldLines = lines;
   lines.clear();
   lineScores.clear();
   //#warning "not grouping at end of getRansacLines"
   mcvGroupLines(newLines, newScores, lineConf->groupThreshold, cvSize(width, height));
   lines = newLines;
   lineScores = newScores;

   //draw splines
   if(DEBUG_LINES) {//#ifdef DEBUG_GET_STOP_LINES

     //get string
     char title[256]; //str[256],
     switch (lineType)
     {
       case LINE_HORIZONTAL:
         sprintf(title, "Lines H");
         break;
       case LINE_VERTICAL:
         sprintf(title, "Lines V");
         break;
     }
     //convert image to rgb
     CvMat* im2 = cvCloneMat(im);
     mcvScaleMat(im2, im2);
     CvMat *imClr = cvCreateMat(im->rows, im->cols, CV_32FC3);
     cvCvtColor(im2, imClr, CV_GRAY2RGB);
     CvMat* imClr2 = cvCloneMat(imClr);
     cvReleaseMat(&im2);

     //draw spline
     for (unsigned int j=0; j<lines.size(); j++)
       mcvDrawLine(imClr, lines[j], CV_RGB(0,1,0), 1);
     SHOW_IMAGE(imClr, title, 10);

     //draw spline
     for (unsigned int j=0; j<oldLines.size(); j++)
       mcvDrawLine(imClr2, oldLines[j], CV_RGB(1,0,0), 1);
     SHOW_IMAGE(imClr2, "Input Lines", 10);

     //clear
     cvReleaseMat(&imClr);
     cvReleaseMat(&imClr2);
     oldLines.clear();
   }//#endif

 //     //put lines back in descending order of scores
 //     lines.clear();
 //     lineScores.clear();
 //     vector<Line>::iterator li;
 //     vector<float>::iterator si;
 //     for (int i=0; i<(int)newLines.size(); i++)
 //     {
   // 	//get its position
   // 	for (li=lines.begin(), si=lineScores.begin();
   // 	     si!=lineScores.end() && newScores[i]<=*si;
   // 	     si++, li++);
   // 	lines.insert(li, newLines[i]);
   // 	lineScores.insert(si, newScores[i]);
   //     }

   //clean
   boxes.clear();
   newLines.clear();
   newScores.clear();
   cvReleaseMat(&image);
 }

 /** This function sets the matrix to a value except for the mask window passed in
  *
  * \param inMat input matrix
  * \param mask the rectangle defining the mask: (xleft, ytop, width, height)
  * \param val the value to put
  */
 void  mcvSetMat(CvMat *inMat, CvRect mask, double val)
 {

   //get x-end points of region to work on, and work on the whole image height
   //(int)fmax(fmin(line.startPoint.x, line.endPoint.x)-xwindow, 0);
   int xstart = mask.x, xend = mask.x + mask.width-1;
   //xend = (int)fmin(fmax(line.startPoint.x, line.endPoint.x), width-1);
   int ystart = mask.y, yend = mask.y + mask.height-1;

   //set other two windows to zero
   CvMat maskMat;
   CvRect rect;
   //part to the left of required region
   rect = cvRect(0, 0, xstart-1, inMat->height);
   if (rect.x<inMat->width && rect.y<inMat->height &&
     rect.x>=0 && rect.y>=0 && rect.width>0 && rect.height>0)
   {
     cvGetSubRect(inMat, &maskMat, rect);
     cvSet(&maskMat, cvRealScalar(val));
   }
   //part to the right of required region
   rect = cvRect(xend+1, 0, inMat->width-xend-1, inMat->height);
   if (rect.x<inMat->width && rect.y<inMat->height &&
     rect.x>=0 && rect.y>=0 && rect.width>0 && rect.height>0)
   {
     cvGetSubRect(inMat, &maskMat, rect);
     cvSet(&maskMat, cvRealScalar(val));
   }

   //part to the top
   rect = cvRect(xstart, 0, mask.width, ystart-1);
   if (rect.x<inMat->width && rect.y<inMat->height &&
     rect.x>=0 && rect.y>=0 && rect.width>0 && rect.height>0)
   {
     cvGetSubRect(inMat, &maskMat, rect);
     cvSet(&maskMat, cvRealScalar(val));
   }

   //part to the bottom
   rect = cvRect(xstart, yend+1, mask.width, inMat->height-yend-1);
   if (rect.x<inMat->width && rect.y<inMat->height &&
     rect.x>=0 && rect.y>=0 && rect.width>0 && rect.height>0)
   {
     cvGetSubRect(inMat, &maskMat, rect);
     cvSet(&maskMat, cvRealScalar(val));
   }
 }

 /** This function binarizes the input image i.e. nonzero elements
  * becomen 1 and others are 0.
  *
  * \param inImage input & output image
  */
 void mcvBinarizeImage(CvMat *inImage)
 {

   if (CV_MAT_TYPE(inImage->type)==FLOAT_MAT_TYPE)
   {
     for (int i=0; i<inImage->height; i++)
       for (int j=0; j<inImage->width; j++)
         if (CV_MAT_ELEM(*inImage, FLOAT_MAT_ELEM_TYPE, i, j) != 0.f)
           CV_MAT_ELEM(*inImage, FLOAT_MAT_ELEM_TYPE, i, j)=1;
   }
   else if (CV_MAT_TYPE(inImage->type)==INT_MAT_TYPE)
   {
     for (int i=0; i<inImage->height; i++)
       for (int j=0; j<inImage->width; j++)
         if (CV_MAT_ELEM(*inImage, INT_MAT_ELEM_TYPE, i, j) != 0)
           CV_MAT_ELEM(*inImage, INT_MAT_ELEM_TYPE, i, j)=1;
   }
   else
   {
     cerr << "Unsupported type in mcvBinarizeImage\n";
     exit(1);
   }
 }

 /** This function groups nearby lines
  *
  * \param lines vector of lines
  * \param lineScores scores of input lines
  * \param groupThreshold the threshold used for grouping
  * \param bbox the bounding box to intersect with
  */
 void mcvGroupLines(vector<Line> &lines, vector<float> &lineScores,
                    float groupThreshold, CvSize bbox)
 {

   //convert the lines into r-theta parameters
   int numInLines = lines.size();
   vector<float> rs(numInLines);
   vector<float> thetas(numInLines);
   for (int i=0; i<numInLines; i++)
     mcvLineXY2RTheta(lines[i], rs[i], thetas[i]);

   //flag for stopping
   bool stop = false;
   while (!stop)
   {
     //minimum distance so far
     float minDist = groupThreshold+5, dist;
     vector<float>::iterator ir, jr, itheta, jtheta, minIr, minJr, minItheta, minJtheta,
     iscore, jscore, minIscore, minJscore;
     //compute pairwise distance between detected maxima
     for (ir=rs.begin(), itheta=thetas.begin(), iscore=lineScores.begin();
     ir!=rs.end(); ir++, itheta++, iscore++)
     for (jr=ir+1, jtheta=itheta+1, jscore=iscore+1;
     jr!=rs.end(); jr++, jtheta++, jscore++)
     {
       //add pi if neg
       float t1 = *itheta<0 ? *itheta : *itheta+CV_PI;
       float t2 = *jtheta<0 ? *jtheta : *jtheta+CV_PI;
       //get distance
       dist = 1 * fabs(*ir - *jr) + 1 * fabs(t1 - t2);//fabs(*itheta - *jtheta);
       //check if minimum
       if (dist<minDist)
       {
         minDist = dist;
         minIr = ir; minItheta = itheta;
         minJr = jr; minJtheta = jtheta;
         minIscore = iscore; minJscore = jscore;
       }
     }
     //check if minimum distance is less than groupThreshold
     if (minDist >= groupThreshold)
       stop = true;
     else
     {
       //put into the first
       *minIr = (*minIr + *minJr)/2;
       *minItheta = (*minItheta + *minJtheta)/2;
       *minIscore = (*minIscore + *minJscore)/2;
       //delete second one
       rs.erase(minJr);
       thetas.erase(minJtheta);
       lineScores.erase(minJscore);
     }
   }//while

   //put back the lines
   lines.clear();
   //lines.resize(rs.size());
   vector<float> newScores=lineScores;
   lineScores.clear();
   for (int i=0; i<(int)rs.size(); i++)
   {
     //get the line
     Line line;
     mcvIntersectLineRThetaWithBB(rs[i], thetas[i], bbox, &line);
     //put in place descendingly
     vector<float>::iterator iscore;
     vector<Line>::iterator iline;
     for (iscore=lineScores.begin(), iline=lines.begin();
     iscore!=lineScores.end() && newScores[i]<=*iscore; iscore++, iline++);
     lineScores.insert(iscore, newScores[i]);
     lines.insert(iline, line);
   }
   //clear
   newScores.clear();
 }

 /** This functions converts a line defined by its two end-points into its
  *   r and theta (origin is at top-left corner with x right and y down and
  * theta measured positive clockwise(with y pointing down) -pi < theta < pi )
  *
  * \param line input line
  * \param r the returned r (normal distance to the line from the origin)
  * \param outLine the output line
  *
  */
 void mcvLineXY2RTheta(const Line &line, float &r, float &theta)
 {
   //check if vertical line x1==x2
   if(line.startPoint.x == line.endPoint.x)
   {
     //r is the x
     r = fabs(line.startPoint.x);
     //theta is 0 or pi
     theta = line.startPoint.x>=0 ? 0. : CV_PI;
   }
   //check if horizontal i.e. y1==y2
   else if(line.startPoint.y == line.endPoint.y)
   {
     //r is the y
     r = fabs(line.startPoint.y);
     //theta is pi/2 or -pi/2
     theta = (float) line.startPoint.y>=0 ? CV_PI/2 : -CV_PI/2;
   }
   //general line
   else
   {
     //tan(theta) = (x2-x1)/(y1-y2)
     theta =  atan2(line.endPoint.x-line.startPoint.x,
                    line.startPoint.y-line.endPoint.y);
     //r = x*cos(theta)+y*sin(theta)
     float r1 = line.startPoint.x * cos(theta) + line.startPoint.y * sin(theta);
     r = line.endPoint.x * cos(theta) + line.endPoint.y * sin(theta);
     //adjust to add pi if necessary
     if(r1<0 || r<0)
     {
       //add pi
       theta += CV_PI;
       if(theta>CV_PI)
         theta -= 2*CV_PI;
       //take abs
       r = fabs(r);
     }
   }
 }

 /** This function intersects the input line (given in r and theta) with
  *  the given bounding box where the line is represented by:
  *  x cos(theta) + y sin(theta) = r
  *
  * \param r the r value for the input line
  * \param theta the theta value for the input line
  * \param bbox the bounding box
  * \param outLine the output line
  *
  */
 void mcvIntersectLineRThetaWithBB(FLOAT r, FLOAT theta, const CvSize bbox,
                                   Line *outLine)
 {
   //hold parameters
   double xup, xdown, yleft, yright;

   //intersect with top and bottom borders: y=0 and y=bbox.height-1
   if (cos(theta)==0) //horizontal line
   {
     xup = xdown = bbox.width+2;
   }
   else
   {
     xup = r / cos(theta);
     xdown = (r-bbox.height*sin(theta))/cos(theta);
   }

   //intersect with left and right borders: x=0 and x=bbox.widht-1
   if (sin(theta)==0) //horizontal line
   {
     yleft = yright = bbox.height+2;
   }
   else
   {
     yleft = r/sin(theta);
     yright = (r-bbox.width*cos(theta))/sin(theta);
   }

   //points of intersection
   FLOAT_POINT2D pts[4] = {{xup, 0},{xdown,bbox.height},
         {0, yleft},{bbox.width, yright}};

   //get the starting point
   int i;
   for (i=0; i<4; i++)
   {
     //if point inside, then put it
     if(mcvIsPointInside(pts[i], bbox))
     {
 	    outLine->startPoint.x = pts[i].x;
 	    outLine->startPoint.y = pts[i].y;
 	    //get out of for loop
 	    break;
     }
   }
   //get the ending point
   for (i++; i<4; i++)
   {
     //if point inside, then put it
     if(mcvIsPointInside(pts[i], bbox))
     {
 	    outLine->endPoint.x = pts[i].x;
 	    outLine->endPoint.y = pts[i].y;
 	    //get out of for loop
 	    break;
     }
   }
 }

 /** \brief This function extracts bounding boxes from lines
  *
  * \param lines vector of lines
  * \param type the type of lines (LINE_HORIZONTAL or LINE_VERTICAL)
  * \param size the size of image containing the lines
  * \param boxes a vector of output bounding boxes
  */
 void mcvGetLinesBoundingBoxes(const vector<Line> &lines, LineType type,
                               CvSize size, vector<CvRect> &boxes)
 {
   //copy lines to boxes
   int start, end;
   //clear
   boxes.clear();
   switch(type)
   {
     case LINE_VERTICAL:
       for(unsigned int i=0; i<lines.size(); ++i)
       {
         //get min and max x and add the bounding box covering the whole height
         start = (int)fmin(lines[i].startPoint.x, lines[i].endPoint.x);
         end = (int)fmax(lines[i].startPoint.x, lines[i].endPoint.x);
         boxes.push_back(cvRect(start, 0, end-start+1, size.height-1));
       }
       break;

     case LINE_HORIZONTAL:
       for(unsigned int i=0; i<lines.size(); ++i)
       {
         //get min and max y and add the bounding box covering the whole width
   	    start = (int)fmin(lines[i].startPoint.y, lines[i].endPoint.y);
         end = (int)fmax(lines[i].startPoint.y, lines[i].endPoint.y);
         boxes.push_back(cvRect(0, start, size.width-1, end-start+1));
       }
       break;
     }
 }

 /** \brief This function extracts bounding boxes including the line inside it
  * \param lines vector of lines
  * \param type the type of lines (LINE_HORIZONTAL or LINE_VERTICAL)
  * \param size the size of image containing the lines
  * \param boxes a vector of output bounding boxes
  */
 void mcvGetLinesBoundingBoxesVec(vector<Line> &lines, LineType type,
                               CvSize size, vector<Box> &boxes)
 {
   //copy lines to boxes
   int start, end;
   SlopeType slopeType;
   //clear
   boxes.clear();
   switch(type)
   {
     case LINE_VERTICAL:
       for(unsigned int i=0; i<lines.size(); ++i)
       {
         //get min and max x and add the bounding box covering the whole height
         //get whether the corresponding line is turning right or not

         //start = (int)fmin(lines[i].startPoint.x, lines[i].endPoint.x);
         if(lines[i].startPoint.x < lines[i].endPoint.x) {
           start = (int)lines[i].startPoint.x;
           end = (int)lines[i].endPoint.x;
           slopeType = (lines[i].endPoint.y - lines[i].startPoint.y) < 0? SLOPE_INCREASING : SLOPE_DECREASING;
         }
         else {
           start = (int)lines[i].endPoint.x;
           end = (int)lines[i].startPoint.x;
           slopeType = (lines[i].startPoint.y - lines[i].endPoint.y) < 0? SLOPE_INCREASING : SLOPE_DECREASING;
         }
         lines[i].slope_type = slopeType;
         //end = (int)fmax(lines[i].startPoint.x, lines[i].endPoint.x);
         boxes.push_back(Box {cvRect(start, 1, end-start+1, size.height-1), lines[i]});
       }
       break;

     //TODO implementation for horizontal lines
     case LINE_HORIZONTAL:
       /*for(unsigned int i=0; i<lines.size(); ++i)
       {
         //get min and max y and add the bounding box covering the whole width
   	    start = (int)fmin(lines[i].startPoint.y, lines[i].endPoint.y);
         end = (int)fmax(lines[i].startPoint.y, lines[i].endPoint.y);
         boxes.push_back(cvRect(0, start, size.width-1, end-start+1));
       }*/
       break;
     }
 }

 /** \brief This function groups together bounding boxes
  *
  * \param size the size of image containing the lines
  * \param boxes a vector of output grouped bounding boxes
  * \param type the type of lines (LINE_HORIZONTAL or LINE_VERTICAL)
  * \param groupThreshold the threshold used for grouping (ratio of overlap)
  */
 void mcvGroupBoundingBoxes(vector<CvRect> &boxes, LineType type,
                            float groupThreshold)
 {
   bool cont = true;

   //Todo: check if to intersect with bounding box or not

   //save boxes
   //vector<CvRect> tboxes = boxes;

   //loop to get the largest overlap (according to type) and check
   //the overlap ratio
   float overlap, maxOverlap;
   while(cont)
   {
     maxOverlap =  overlap = -1e5;
     //loop on lines and get max overlap
     vector<CvRect>::iterator i, j, maxI, maxJ;
     for(i = boxes.begin(); i != boxes.end(); i++)
     {
       for(j = i+1; j != boxes.end(); j++)
       {
         switch(type)
         {
           case LINE_VERTICAL:
             //get one with smallest x, and compute the x2 - x1 / width of smallest
             //i.e. (x12 - x21) / (x22 - x21)
             overlap = i->x < j->x  ?
             (i->x + i->width - j->x) / (float)j->width :
             (j->x + j->width - i->x) / (float)i->width;

             break;

           case LINE_HORIZONTAL:
             //get one with smallest y, and compute the y2 - y1 / height of smallest
             //i.e. (y12 - y21) / (y22 - y21)
             overlap = i->y < j->y  ?
             (i->y + i->height - j->y) / (float)j->height :
             (j->y + j->height - i->y) / (float)i->height;

             break;

         } //switch

         //get maximum
         if(overlap > maxOverlap)
         {
           maxI = i;
           maxJ = j;
           maxOverlap = overlap;
         }
       } //for j
     } // for i
     // 	//debug
     // 	if(DEBUG_LINES) {
     // 	    cout << "maxOverlap=" << maxOverlap << endl;
     // 	    cout << "Before grouping\n";
     // 	    for(unsigned int k=0; k<boxes.size(); ++k)
     // 		SHOW_RECT(boxes[k]);
     // 	}

     //now check the max overlap found against the threshold
     if (maxOverlap >= groupThreshold)
     {
       //combine the two boxes
       *maxI  = cvRect(min((*maxI).x, (*maxJ).x),
                       min((*maxI).y, (*maxJ).y),
                       max((*maxI).width, (*maxJ).width),
                       max((*maxI).height, (*maxJ).height));
                       //delete the second one
                       boxes.erase(maxJ);
     }
     else
       //stop
       cont = false;

     // 	//debug
     // 	if(DEBUG_LINES) {
     // 	    cout << "After grouping\n";
     // 	    for(unsigned int k=0; k<boxes.size(); ++k)
     // 		SHOW_RECT(boxes[k]);
     // 	}
   } //while
 }

 /** \brief This function groups together bounding boxes (with the line inside it)
  *
  * \param size the size of image containing the lines
  * \param boxes a vector of output grouped bounding boxes
  * \param type the type of lines (LINE_HORIZONTAL or LINE_VERTICAL)
  * \param groupThreshold the threshold used for grouping (ratio of overlap)
  */
 void mcvGroupBoundingBoxesVec(vector<Box> &boxes, LineType type,
                            float groupThreshold)
 {
   bool cont = true;

   //Todo: check if to intersect with bounding box or not

   //save boxes
   //vector<CvRect> tboxes = boxes;

   //loop to get the largest overlap (according to type) and check
   //the overlap ratio
   float overlap, maxOverlap;
   while(cont)
   {
     maxOverlap =  overlap = -1e5;
     //loop on lines and get max overlap
     vector<Box>::iterator i, j, maxI, maxJ;
     for(i = boxes.begin(); i != boxes.end(); i++)
     {
       for(j = i+1; j != boxes.end(); j++)
       {
         switch(type)
         {
           case LINE_VERTICAL:
             //get one with smallest x, and compute the x2 - x1 / width of smallest
             //i.e. (x12 - x21) / (x22 - x21)
             overlap = i->box.x < j->box.x  ?
             (i->box.x + i->box.width - j->box.x) / (float)j->box.width :
             (j->box.x + j->box.width - i->box.x) / (float)i->box.width;

             break;

           case LINE_HORIZONTAL:
             //get one with smallest y, and compute the y2 - y1 / height of smallest
             //i.e. (y12 - y21) / (y22 - y21)
             overlap = i->box.y < j->box.y  ?
             (i->box.y + i->box.height - j->box.y) / (float)j->box.height :
             (j->box.y + j->box.height - i->box.y) / (float)i->box.height;

             break;

         } //switch

         //get maximum
         if(overlap > maxOverlap)
         {
           maxI = i;
           maxJ = j;
           maxOverlap = overlap;
         }
       } //for j
     } // for i
     // 	//debug
     // 	if(DEBUG_LINES) {
     // 	    cout << "maxOverlap=" << maxOverlap << endl;
     // 	    cout << "Before grouping\n";
     // 	    for(unsigned int k=0; k<boxes.size(); ++k)
     // 		SHOW_RECT(boxes[k]);
     // 	}

     //now check the max overlap found against the threshold
     if (maxOverlap >= groupThreshold)
     {
       //Take the line with best score
       Line lin = (*maxI).line.score > (*maxJ).line.score ? (*maxI).line : (*maxJ).line;
       //combine the two boxes
       *maxI  = Box{cvRect(min((*maxI).box.x, (*maxJ).box.x),
                       min((*maxI).box.y, (*maxJ).box.y),
                       max((*maxI).box.width, (*maxJ).box.width),
                       max((*maxI).box.height, (*maxJ).box.height)), lin};
                       //delete the second one
                       boxes.erase(maxJ);
     }
     else
       //stop
       cont = false;

     // 	//debug
     // 	if(DEBUG_LINES) {
     // 	    cout << "After grouping\n";
     // 	    for(unsigned int k=0; k<boxes.size(); ++k)
     // 		SHOW_RECT(boxes[k]);
     // 	}
   } //while
 }

 /** This functions implements RANSAC algorithm for line fitting
  *   given an image
  *
  *
  * \param image input image
  * \param numSamples number of samples to take every iteration
  * \param numIterations number of iterations to run
  * \param threshold threshold to use to assess a point as a good fit to a line
  * \param numGoodFit number of points close enough to say there's a good fit
  * \param getEndPoints whether to get the end points of the line from the data,
  *  just intersect with the image boundaries
  * \param lineType the type of line to look for (affects getEndPoints)
  * \param lineXY the fitted line
  * \param lineRTheta the fitted line [r; theta]
  * \param lineScore the score of the line detected
  *
  */
 void mcvFitRansacLine(const CvMat *image, int numSamples, int numIterations,
                       float threshold, float scoreThreshold, int numGoodFit,
                       bool getEndPoints, LineType lineType,
                       Line *lineXY, float *lineRTheta, float *lineScore)
 {

   //get the points with non-zero pixels
   CvMat *points;
   points = mcvGetNonZeroPoints(image,true);
   if (!points)
     return;
   //check numSamples
   if (numSamples>points->cols)
     numSamples = points->cols;
   //subtract half
   cvAddS(points, cvRealScalar(0.5), points);

   //normalize pixels values to get weights of each non-zero point
   //get third row of points containing the pixel values
   CvMat w;
   cvGetRow(points, &w, 2);
   //normalize it
   CvMat *weights = cvCloneMat(&w);
   cvNormalize(weights, weights, 1, 0, CV_L1);
   //get cumulative    sum
   mcvCumSum(weights, weights);

   //random number generator
   CvRNG rng = cvRNG(0xffffffff);
   //matrix to hold random sample
   CvMat *randInd = cvCreateMat(numSamples, 1, CV_32SC1);
   CvMat *samplePoints = cvCreateMat(2, numSamples, CV_32FC1);
   //flag for points currently included in the set
   CvMat *pointIn = cvCreateMat(1, points->cols, CV_8SC1);
   //returned lines
   float curLineRTheta[2], curLineAbc[3];
   float bestLineRTheta[2]={-1.f,0.f}, bestLineAbc[3];
   float bestScore=0, bestDist=1e5;
   float dist, score;
   Line curEndPointLine={{-1.,-1.},{-1.,-1.}},
   bestEndPointLine={{-1.,-1.},{-1.,-1.}};
   //variabels for getting endpoints
   //int mini, maxi;
   float minc=1e5f, maxc=-1e5f, mind, maxd;
   float x, y, c=0.;
   CvPoint2D32f minp={-1., -1.}, maxp={-1., -1.};
   //outer loop
   for (int i=0; i<numIterations; i++)
   {
     //set flag to zero
     //cvSet(pointIn, cvRealScalar(0));
     cvSetZero(pointIn);
     //get random sample from the points
     #warning "Using weighted sampling for Ransac Line"
     // 	cvRandArr(&rng, randInd, CV_RAND_UNI, cvRealScalar(0), cvRealScalar(points->cols));
     mcvSampleWeighted(weights, numSamples, randInd, &rng);

     for (int j=0; j<numSamples; j++)
     {
       //flag it as included
       CV_MAT_ELEM(*pointIn, char, 0, CV_MAT_ELEM(*randInd, int, j, 0)) = 1;
       //put point
       CV_MAT_ELEM(*samplePoints, float, 0, j) =
       CV_MAT_ELEM(*points, float, 0, CV_MAT_ELEM(*randInd, int, j, 0));
       CV_MAT_ELEM(*samplePoints, float, 1, j) =
       CV_MAT_ELEM(*points, float, 1, CV_MAT_ELEM(*randInd, int, j, 0));
     }

     //fit the line
     mcvFitRobustLine(samplePoints, curLineRTheta, curLineAbc);

     //get end points from points in the samplePoints
     minc = 1e5; mind = 1e5; maxc = -1e5; maxd = -1e5;
     for (int j=0; getEndPoints && j<numSamples; ++j)
     {
       //get x & y
       x = CV_MAT_ELEM(*samplePoints, float, 0, j);
       y = CV_MAT_ELEM(*samplePoints, float, 1, j);

       //get the coordinate to work on
       if (lineType == LINE_HORIZONTAL)
         c = x;
       else if (lineType == LINE_VERTICAL)
         c = y;
       //compare
       if (c>maxc)
       {
         maxc = c;
         maxp = cvPoint2D32f(x, y);
       }
       if (c<minc)
       {
         minc = c;
         minp = cvPoint2D32f(x, y);
       }
     } //for

     // 	fprintf(stderr, "\nminx=%f, miny=%f\n", minp.x, minp.y);
     // 	fprintf(stderr, "maxp=%f, maxy=%f\n", maxp.x, maxp.y);

     //loop on other points and compute distance to the line
     score=0;
     for (int j=0; j<points->cols; j++)
     {
       // 	    //if not already inside
       // 	    if (!CV_MAT_ELEM(*pointIn, char, 0, j))
       // 	    {
         //compute distance to line
         dist = fabs(CV_MAT_ELEM(*points, float, 0, j) * curLineAbc[0] +
         CV_MAT_ELEM(*points, float, 1, j) * curLineAbc[1] + curLineAbc[2]);
         //check distance
         if (dist<=threshold)
         {
           //add this point
           CV_MAT_ELEM(*pointIn, char, 0, j) = 1;
           //update score
           score += cvGetReal2D(image, (int)(CV_MAT_ELEM(*points, float, 1, j)-.5),
                                (int)(CV_MAT_ELEM(*points, float, 0, j)-.5));
         }
         // 	    }
     }

     //check the number of close points and whether to consider this a good fit
     int numClose = cvCountNonZero(pointIn);
     //cout << "numClose=" << numClose << "\n";
     if (numClose >= numGoodFit)
     {
         //get the points included to fit this line
         CvMat *fitPoints = cvCreateMat(2, numClose, CV_32FC1);
         int k=0;
         //loop on points and copy points included
         for (int j=0; j<points->cols; j++)
       if(CV_MAT_ELEM(*pointIn, char, 0, j))
       {
           CV_MAT_ELEM(*fitPoints, float, 0, k) =
         CV_MAT_ELEM(*points, float, 0, j);
           CV_MAT_ELEM(*fitPoints, float, 1, k) =
         CV_MAT_ELEM(*points, float, 1, j);
           k++;

       }

       //fit the line
       mcvFitRobustLine(fitPoints, curLineRTheta, curLineAbc);

       //compute distances to new line
       dist = 0.;
       for (int j=0; j<fitPoints->cols; j++)
       {
         //compute distance to line
         x = CV_MAT_ELEM(*fitPoints, float, 0, j);
         y = CV_MAT_ELEM(*fitPoints, float, 1, j);
         float d = fabs( x * curLineAbc[0] +
         y * curLineAbc[1] +
         curLineAbc[2])
         * cvGetReal2D(image, (int)(y-.5), (int)(x-.5));
         dist += d;

         // 		//check min and max coordinates to get extent
         // 		if (getEndPoints)
         // 		{
           // 		    //get the coordinate to work on
           // 		    if (lineType == LINE_HORIZONTAL)
           // 			c = x;
           // 		    else if (lineType == LINE_VERTICAL)
           // 			c = y;
           // 		    //compare
           // 		    if (c>maxc)
           // 		    {
             // 			maxc = c;
             // 			maxd = d;
             // 			maxp = cvPoint2D32f(x, y);
             // 		    }
             // 		    if (c<minc)
             // 		    {
               // 			minc = c;
               // 			mind = d;
               // 			minp = cvPoint2D32f(x, y);
               // 		    }

               // // 		    fprintf(stderr, "minc=%f, mind=%f, mini=%d\n", minc, mind, mini);
               // // 		    fprintf(stderr, "maxc=%f, maxd=%f, maxi=%d\n", maxc, maxd, maxi);
               // 		}
       }

       //now check if we are getting the end points
       if (getEndPoints)
       {

         //get distances
         mind = minp.x * curLineAbc[0] +
         minp.y * curLineAbc[1] + curLineAbc[2];
         maxd = maxp.x * curLineAbc[0] +
         maxp.y * curLineAbc[1] + curLineAbc[2];

         //we have the index of min and max points, and
         //their distance, so just get them and compute
         //the end points
         curEndPointLine.startPoint.x = minp.x
         - mind * curLineAbc[0];
         curEndPointLine.startPoint.y = minp.y
         - mind * curLineAbc[1];

         curEndPointLine.endPoint.x = maxp.x
         - maxd * curLineAbc[0];
         curEndPointLine.endPoint.y = maxp.y
         - maxd * curLineAbc[1];

         // 		SHOW_MAT(fitPoints, "fitPoints");
         //  		SHOW_LINE(curEndPointLine, "line");
       }

       //dist /= score;

       //clear fitPoints
       cvReleaseMat(&fitPoints);

       //check if to keep the line as best
       if (score>=scoreThreshold && score>bestScore)//dist<bestDist //(numClose > bestScore)
       {
         //update max
         bestScore = score; //numClose;
         bestDist = dist;
         //copy
         bestLineRTheta[0] = curLineRTheta[0];
         bestLineRTheta[1] = curLineRTheta[1];
         bestLineAbc[0] = curLineAbc[0];
         bestLineAbc[1] = curLineAbc[1];
         bestLineAbc[2] = curLineAbc[2];
         bestEndPointLine = curEndPointLine;
       }
     } // if numClose

     //debug
     if (DEBUG_LINES) {//#ifdef DEBUG_GET_STOP_LINES
       char str[256];
       //convert image to rgb
       CvMat* im = cvCloneMat(image);
       mcvScaleMat(image, im);
       CvMat *imageClr = cvCreateMat(image->rows, image->cols, CV_32FC3);
       cvCvtColor(im, imageClr, CV_GRAY2RGB);

       Line line;
       //draw current line if there
       if (curLineRTheta[0]>0)
       {
         mcvIntersectLineRThetaWithBB(curLineRTheta[0], curLineRTheta[1],
                                     cvSize(image->cols, image->rows), &line);
         mcvDrawLine(imageClr, line, CV_RGB(1,0,0), 1);
         if (getEndPoints)
           mcvDrawLine(imageClr, curEndPointLine, CV_RGB(0,1,0), 1);
       }

       //draw best line
       if (bestLineRTheta[0]>0)
       {
         mcvIntersectLineRThetaWithBB(bestLineRTheta[0], bestLineRTheta[1],
                                     cvSize(image->cols, image->rows), &line);
         mcvDrawLine(imageClr, line, CV_RGB(0,0,1), 1);
         if (getEndPoints)
           mcvDrawLine(imageClr, bestEndPointLine, CV_RGB(1,1,0), 1);
       }
       sprintf(str, "scor=%.2f, best=%.2f", score, bestScore);
       mcvDrawText(imageClr, str, cvPoint(30, 30), .25, CV_RGB(255,255,255));

       SHOW_IMAGE(imageClr, "Fit Ransac Line", 10);

       //clear
       cvReleaseMat(&im);
       cvReleaseMat(&imageClr);
     }//#endif
   } // for i

   //return
   if (lineRTheta)
   {
     lineRTheta[0] = bestLineRTheta[0];
     lineRTheta[1] = bestLineRTheta[1];
   }
   if (lineXY)
   {
     if (getEndPoints)
       *lineXY = bestEndPointLine;
     else
       mcvIntersectLineRThetaWithBB(lineRTheta[0], lineRTheta[1],
                                    cvSize(image->cols-1, image->rows-1),
                                    lineXY);
   }
   if (lineScore)
     *lineScore = bestScore;

   //clear
   cvReleaseMat(&points);
   cvReleaseMat(&samplePoints);
   cvReleaseMat(&randInd);
   cvReleaseMat(&pointIn);
 }

 /** This function gets the indices of the non-zero values in a matrix
  *
  * \param inMat the input matrix
  * \param outMat the output matrix, with 2xN containing the x and y in
  *    each column and the pixels value [xs; ys; pixel values]
  * \param floatMat whether to return floating points or integers for
  *    the outMat
  */
 CvMat* mcvGetNonZeroPoints(const CvMat *inMat, bool floatMat)
 {

 #define MCV_GET_NZ_POINTS(inMatType, outMatType) \
      /*loop and allocate the points*/ \
      for (int i=0; i<inMat->rows; i++) \
  	for (int j=0; j<inMat->cols; j++) \
  	    if (CV_MAT_ELEM(*inMat, inMatType, i, j)) \
  	    { \
  		CV_MAT_ELEM(*outMat, outMatType, 0, k) = j; \
  		CV_MAT_ELEM(*outMat, outMatType, 1, k) = i; \
                 CV_MAT_ELEM(*outMat, outMatType, 2, k) = \
                   (outMatType) CV_MAT_ELEM(*inMat, inMatType, i, j); \
                 k++; \
  	    } \

   int k=0;

   //get number of non-zero points
   int numnz = cvCountNonZero(inMat);

   //allocate the point array and get the points
   CvMat* outMat;
   if (numnz)
   {
     if (floatMat)
       outMat = cvCreateMat(3, numnz, CV_32FC1);
     else
       outMat = cvCreateMat(3, numnz, CV_32SC1);
   }
   else
     return NULL;

   //check type
   if (CV_MAT_TYPE(inMat->type)==FLOAT_MAT_TYPE &&
     CV_MAT_TYPE(outMat->type)==FLOAT_MAT_TYPE)
   {
     MCV_GET_NZ_POINTS(FLOAT_MAT_ELEM_TYPE, FLOAT_MAT_ELEM_TYPE)
   }
   else if (CV_MAT_TYPE(inMat->type)==FLOAT_MAT_TYPE &&
     CV_MAT_TYPE(outMat->type)==INT_MAT_TYPE)
   {
     MCV_GET_NZ_POINTS(FLOAT_MAT_ELEM_TYPE, INT_MAT_ELEM_TYPE)
   }
   else if (CV_MAT_TYPE(inMat->type)==INT_MAT_TYPE &&
     CV_MAT_TYPE(outMat->type)==FLOAT_MAT_TYPE)
   {
     MCV_GET_NZ_POINTS(INT_MAT_ELEM_TYPE, FLOAT_MAT_ELEM_TYPE)
   }
   else if (CV_MAT_TYPE(inMat->type)==INT_MAT_TYPE &&
     CV_MAT_TYPE(outMat->type)==INT_MAT_TYPE)
   {
     MCV_GET_NZ_POINTS(INT_MAT_ELEM_TYPE, INT_MAT_ELEM_TYPE)
   }
   else
   {
     cerr << "Unsupported type in mcvGetMatLocalMax\n";
     exit(1);
   }

   //return
   return outMat;
 }

 /** This function computes the cumulative sum for a vector
  *
  * \param inMat input matrix
  * \param outMat output matrix
  *
  */
 void mcvCumSum(const CvMat *inMat, CvMat *outMat)
 {

 #define MCV_CUM_SUM(type) 				\
     /*row vector*/ 					\
     if(inMat->rows == 1) 				\
 	for (int i=1; i<outMat->cols; i++) 		\
 	    CV_MAT_ELEM(*outMat, type, 0, i) += 	\
 		CV_MAT_ELEM(*outMat, type, 0, i-1); 	\
     /*column vector*/					\
     else						\
 	for (int i=1; i<outMat->rows; i++) 		\
 	    CV_MAT_ELEM(*outMat, type, i, 0) += 	\
 		CV_MAT_ELEM(*outMat, type, i-1, 0);

   //copy to output if not equal
   if(inMat != outMat)
     cvCopy(inMat, outMat);

   //check type
   if (CV_MAT_TYPE(inMat->type)==CV_32FC1)
   {
     MCV_CUM_SUM(float)
   }
   else if (CV_MAT_TYPE(inMat->type)==CV_32SC1)
   {
     MCV_CUM_SUM(int)
   }
   else
   {
     cerr << "Unsupported type in mcvCumSum\n";
     exit(1);
   }
 }

 /** This function samples uniformly with weights
  *
  * \param cumSum cumulative sum for normalized weights for the differnet
  *          samples (last is 1)
  * \param numSamples the number of samples
  * \param randInd a 1XnumSamples of int containing the indices
  * \param rng a pointer to a random number generator
  *
  */
 void mcvSampleWeighted(const CvMat *cumSum, int numSamples, CvMat *randInd,
                        CvRNG *rng)
 {
 //     //get cumulative sum of the weights
 //     //OPTIMIZE:should pass it later instead of recomputing it
 //     CvMat *cumSum = cvCloneMat(weights);
 //     for (int i=1; i<weights->cols; i++)
 // 	CV_MAT_ELEM(*cumSum, float, 0, i) += CV_MAT_ELEM(*cumSum, float, 0, i-1);

   //check if numSamples is equal or more
   int i=0;
   if (numSamples >= cumSum->cols)
   {
     for (; i<numSamples; i++)
       CV_MAT_ELEM(*randInd, int, i, 0) = i;
   }
   else
   {
     //loop
     while(i<numSamples)
     {
       //get random number
       double r = cvRandReal(rng);

       //get the index from cumSum
       int j;
       for (j=0; j<cumSum->cols && r>CV_MAT_ELEM(*cumSum, float, 0, j); j++);

       //make sure this index wasnt chosen before
       bool put = true;
       for (int k=0; k<i; k++)
         if (CV_MAT_ELEM(*randInd, int, k, 0) == j)
           //put it
           put = false;

       if (put)
       {
         //put it in array
         CV_MAT_ELEM(*randInd, int, i, 0) = j;
         //inc
         i++;
       }
     } //while
   } //if
 }

 /** This functions fits a line using the orthogonal distance to the line
     by minimizing the sum of squares of this distance.

  *
  * \param points the input points to fit the line to which is
  *    2xN matrix with x values on first row and y values on second
  * \param lineRTheta the return line [r, theta] where the line is
  *    x*cos(theta)+y*sin(theta)=r
  * \param lineAbc the return line in [a, b, c] where the line is
  *    a*x+b*y+c=0
  *
  */
 void mcvFitRobustLine(const CvMat *points, float *lineRTheta,
                       float *lineAbc)
 {
   // check number of points
   if (points->cols < 2)
   {
     return;
   }

   //clone the points
   CvMat *cpoints = cvCloneMat(points);
   //get mean of the points and subtract from the original points
   float meanX=0, meanY=0;
   CvScalar mean;
   CvMat row1, row2;
   //get first row, compute avg and store
   cvGetRow(cpoints, &row1, 0);
   mean = cvAvg(&row1);
   meanX = (float) mean.val[0];
   cvSubS(&row1, mean, &row1);
   //same for second row
   cvGetRow(cpoints, &row2, 1);
   mean = cvAvg(&row2);
   meanY = (float) mean.val[0];
   cvSubS(&row2, mean, &row2);

   //compute the SVD for the centered points array
   CvMat *W = cvCreateMat(2, 1, CV_32FC1);
   CvMat *V = cvCreateMat(2, 2, CV_32FC1);
   //    CvMat *V = cvCreateMat(2, 2, CV_32fC1);
   CvMat *cpointst = cvCreateMat(cpoints->cols, cpoints->rows, CV_32FC1);

   cvTranspose(cpoints, cpointst);
   cvSVD(cpointst, W, 0, V, CV_SVD_V_T);
   cvTranspose(V, V);
   cvReleaseMat(&cpointst);

   //get the [a,b] which is the second column corresponding to
   //smaller singular value
   float a, b, c;
   a = CV_MAT_ELEM(*V, float, 0, 1);
   b = CV_MAT_ELEM(*V, float, 1, 1);

   //c = -meanX*a-meanY*b
   c = -(meanX * a + meanY * b);

   //compute r and theta
   //theta = atan(b/a)
   //r = meanX cos(theta) + meanY sin(theta)
   float r, theta;
   theta = atan2(b, a);
   r = meanX * cos(theta) + meanY * sin(theta);
   //correct
   if (r<0)
   {
     //correct r
     r = -r;
     //correct theta
     theta += CV_PI;
     if (theta>CV_PI)
       theta -= 2*CV_PI;
   }
   //return
   if (lineRTheta)
   {
     lineRTheta[0] = r;
     lineRTheta[1] = theta;
   }
   if (lineAbc)
   {
     lineAbc[0] = a;
     lineAbc[1] = b;
     lineAbc[2] = c;
   }
   //clear
   cvReleaseMat(&cpoints);
   cvReleaseMat(&W);
   cvReleaseMat(&V);
 }

 /** This function draws a line onto the passed image
  *
  * \param image the input iamge
  * \param line input line
  * \param line color
  * \param width line width
  *
  */
 void mcvDrawLine(CvMat *image, Line line, CvScalar color, int width)
 {
   cvLine(image, cvPoint((int)line.startPoint.x,(int)line.startPoint.y),
           cvPoint((int)line.endPoint.x,(int)line.endPoint.y),
           color, width);
 }

 /** This function draws a rectangle onto the passed image
  *
  * \param image the input iamge
  * \param rect the input rectangle
  * \param color the rectangle color
  * \param width the rectangle width
  *
  */
 void mcvDrawRectangle (CvMat *image, CvRect rect, CvScalar color, int width)
 {
   //draw the rectangle
   cvRectangle(image, cvPoint(rect.x, rect.y),
               cvPoint(rect.x + rect.width-1, rect.y + rect.height-1),
               color, width);

 }

 /** This function draws a spline onto the passed image
  *
  * \param image the input iamge
  * \param str the string to put
  * \param point the point where to put the text
  * \param size the font size
  * \param color the font color
  *
  */
 void mcvDrawText(CvMat *image, char* str, CvPoint point,
 		 float size, CvScalar color)
 {

   CvFont font;
   cvInitFont(&font, CV_FONT_HERSHEY_TRIPLEX, size, size);
   cvPutText(image, str, point, &font, color);

 }

} // namespace LaneDetector
