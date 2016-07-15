#include <lane_detector/fitting.h>
#include <opencv2/highgui/highgui.hpp>
#include <swri_profiler/profiler.h>

void Fitting::fitting(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& ipmBoxes)
{

        SWRI_PROFILE("Fitting");

        FittingApproach fitSpline;

        std::vector<cv::Point> splinePoints;
        std::vector< std::vector<cv::Point> > splines;
        std::vector<cv::Point> centroids;
        cv::Point car_position(200, lanesConf.ipmHeight-1);

        for(LaneDetector::Box box : ipmBoxes) {
          cv::Rect bounding_box = box.box;
          fitSpline.fitting(preprocessed, bounding_box, splinePoints);
          splines.push_back(splinePoints);
          const cv::Point *pts = (const cv::Point*) cv::Mat(splinePoints).data;
	        int npts = cv::Mat(splinePoints).rows;

          // draw the spline
	         if(config.draw_splines)cv::polylines(original, &pts,&npts, 1,
            	    		false, 			// draw open contour
            	            cv::Scalar(255, 0, 0),// colour RGB ordering (here = green)
            	    		2 		        // line thickness
                      );

          int centroid_x = cvRound((box.box.x + (box.box.x + box.box.width-1))/2);
          int centroid_y = cvRound((box.box.y + (box.box.y + box.box.height-1))/2);
          cv::Point centroid(centroid_x, centroid_y);
          centroids.push_back(centroid);
          cv::circle(original, cv::Point(centroid_x, centroid_y), 2, cv::Scalar(0,255,0), 2);
          if(config.draw_boxes) cv::rectangle(original, cv::Point(box.box.x, box.box.y),
                                                  cv::Point(box.box.x + box.box.width-1, box.box.y + box.box.height-1),
                                                  cv::Scalar(0,0,255));

          splinePoints.clear();
        }

        //std::sort(.begin(), boxes.end(), lane_detector::utils::compareBoxes);
        cv::Point closest;
        cv::Point second_closest;
        int closest_idx = 0;
        int second_closest_idx = 0;;
        if(centroids.size() > 2) {
          closest = centroids[0];
          second_closest = centroids[1];
          closest_idx = 0;
          second_closest_idx = 1;
          int min_distance = std::abs(closest.x - car_position.x);
          int second_min_distance = std::abs(second_closest.x - car_position.x);
          for(int i = 2; i < centroids.size(); i++) {
            int current_distance = std::abs(centroids[i].x - car_position.x);
            if(current_distance <= min_distance) {
              second_min_distance = min_distance;
              min_distance = current_distance;
              second_closest = closest;
              closest = centroids[i];
              second_closest_idx = closest_idx;
              closest_idx = i;
            }
          }
        }
        else if(centroids.size() == 2) {
          closest = centroids[0];
          second_closest = centroids[1];
          closest_idx = 0;
          second_closest_idx = 1;
        }

      /*  cv::circle(original, cv::Point(last_left_centroid.x, last_left_centroid.y), 2, cv::Scalar(0,0,255), 2);
        cv::circle(original, cv::Point(last_right_centroid.x, last_right_centroid.y), 2, cv::Scalar(0,0,255), 2);
        cv::line(original, cv::Point(car_position.x, 0), cv::Point(car_position.x, lanesConf.ipmHeight-1), cv::Scalar(0, 255, 239), 1);*/
}
