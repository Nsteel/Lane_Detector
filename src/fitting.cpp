#include <lane_detector/fitting.h>
#include <swri_profiler/profiler.h>

void Fitting::fitting(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Line>& lines)
{

        SWRI_PROFILE("Fitting");

        FittingApproach fitSpline;
        std::vector<LaneDetector::Box> ipmBoxes;

        mcvGetLinesBoundingBoxesWithSlope(lines, LaneDetector::LINE_VERTICAL, cvSize(preprocessed.cols-1, preprocessed.rows-1), ipmBoxes);
        utils::resizeBoxes(ipmBoxes, LaneDetector::LINE_VERTICAL);

        cv::Mat aux = preprocessed.clone();
        std::vector<cv::Point> splinePoints;
        int i = 0;
        for(LaneDetector::Box box : ipmBoxes) {
          cv::Mat roi = preprocessed(box.box);
          fitSpline.fitting(roi, splinePoints);
          const cv::Point *pts = (const cv::Point*) cv::Mat(splinePoints).data;
	        int npts = cv::Mat(splinePoints).rows;

          int b = i==0 ? 255 : 0;
          int g = i==1 ? 255 : 0;
          int r = i==2 ? 255 : 0;
          i++;
          // draw the spline
	         cv::polylines(roi, &pts,&npts, 1,
            	    		false, 			// draw open contour
            	            cv::Scalar(b,g,r),// colour RGB ordering (here = green)
            	    		2 		        // line thickness
                      );
          roi.copyTo(aux(box.box));
          splinePoints.clear();
          if(config.draw_boxes) cv::rectangle(aux, cv::Point(box.box.x, box.box.y),
                                                  cv::Point(box.box.x + box.box.width-1, box.box.y + box.box.height-1),
                                                  cv::Scalar(0,0,255));
        }
        preprocessed = aux;
}
