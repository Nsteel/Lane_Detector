#include <lane_detector/fitting.h>
#include <opencv2/highgui/highgui.hpp>
#include <swri_profiler/profiler.h>


void Fitting::fitting(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& ipmBoxes)
{

        SWRI_PROFILE("Fitting");
        cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };
        FittingApproach fitSpline;

        std::vector<cv::Point> splinePoints;
        std::vector< std::vector<cv::Point> > splines;
        std::vector<cv::Point2f> centroids;
        std::vector<cv::Rect> rects;
        cv::Point car_position(200, lanesConf.ipmHeight-1);
        lane_detector::utils::boxes2Rects(ipmBoxes, rects);
        lane_detector::utils::getRectsCentroids(rects, centroids);
        tracker.Update(centroids, rects, CTracker::RectsDist);
        rects = tracker.getLastRects();
        lane_detector::utils::getRectsCentroids(rects, centroids);

        //std::cout << "Lanes Detected: " << tracker.tracks.size() << std::endl;

        for(cv::Rect bounding_box : rects) {
          fitSpline.fitting(preprocessed, bounding_box, splinePoints);
          std::sort(splinePoints.begin(), splinePoints.end(), lane_detector::utils::sortPointsY);
          splines.push_back(splinePoints);

          // draw the spline
	        if(config.draw_splines) lane_detector::utils::drawSpline(original, splinePoints, cv::Scalar(255, 0, 0));

          if(config.draw_boxes) cv::rectangle(original, cv::Point(bounding_box.x, bounding_box.y),
                                                  cv::Point(bounding_box.x + bounding_box.width-1, bounding_box.y + bounding_box.height-1),
                                                  cv::Scalar(0,0,255));

          splinePoints.clear();
        }

        cv::Point closest;
        cv::Point second_closest;
        int closest_idx = 0;
        int second_closest_idx = 0;
        //std::cout << "centroids: " << centroids.size() << std::endl;
        if(centroids.size() >= 2) {
          int distance_0 = std::abs(centroids[0].x - car_position.x);
          int distance_1 = std::abs(centroids[1].x - car_position.x);
          if(distance_0 < distance_1) {
            closest = centroids[0];
            second_closest = centroids[1];
            closest_idx = 0;
            second_closest_idx = 1;
          }
          else {
            closest = centroids[1];
            second_closest = centroids[0];
            closest_idx = 1;
            second_closest_idx = 0;
          }
          int min_distance = std::min(distance_0, distance_1);
          int second_min_distance = std::max(distance_0, distance_1);
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
            else if(current_distance < second_min_distance){
              second_closest = centroids[i];
              second_closest_idx = i;
              second_min_distance = current_distance;
            }
          }
        }
        //std::cout << "splines size: " << splines.size() << std::endl;
        if(splines.size() == 3) {
          //std::cout << "1: " << splines[0].size() << " 2: " << splines[1].size() << " 3: " << splines[2].size() << std::endl;
        }

        cv::circle(original, closest, 5, cv::Scalar(0,255,0), 2);
        cv::circle(original, second_closest, 5, cv::Scalar(0,255,0), 2);

        for(int i = 0; i < tracker.tracks.size(); i++) {
          if (tracker.tracks[i]->trace.size() >= 1)
          {
          cv::circle(original, tracker.tracks[i]->trace.back(), 2, Colors[tracker.tracks[i]->track_id % 9], 2);
          }
        }

        std::vector<cv::Point> longest_spline;
        std::vector<cv::Point> second_longest_spline;

        //std::cout << "splines: " << splines.size() << std::endl;
        //if(splines.size() > 1) std::cout << "closest: " << splines[closest_idx].size() << " second: " << splines[second_closest_idx].size() << std::endl;

        if(splines.size() > 1 && splines[closest_idx].size() > 3 && splines[second_closest_idx].size() > 3) {
          if(splines[closest_idx][0].y < splines[second_closest_idx][0].y)  {
            longest_spline = splines[closest_idx];
            second_longest_spline = splines[second_closest_idx];
          }
          else {
            longest_spline = splines[second_closest_idx];
            second_longest_spline = splines[closest_idx];
          }

          int middle = 0;
          for(int i = 0; i < longest_spline.size(); i++) {
            if(longest_spline[i].y >= second_longest_spline[0].y) {
              middle = cvRound((longest_spline[i].x - second_longest_spline[0].x)/2);

              //std::cout << "Middle: " << middle << std::endl;
              break;
            }
          }

          for(int i = 0; i < longest_spline.size(); i++) {
            longest_spline[i].x -= middle;
          }

          // draw the guide spline
           if(config.draw_splines) lane_detector::utils::drawSpline(original, longest_spline, cv::Scalar(0, 255, 255));


        }

      cv::line(original, cv::Point(car_position.x, 0), cv::Point(car_position.x, lanesConf.ipmHeight-1), cv::Scalar(0, 255, 239), 1);
}
