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

        if(!kf_initialized && ipmBoxes.size() == 3) {
          initTracking(ipmBoxes);
          kf_initialized = true;
        }

        if(kf_initialized) trackBoxes(ipmBoxes);

        for(LaneDetector::Box box : ipmBoxes) {
          cv::Rect bounding_box = box.box;
          fitSpline.fitting(preprocessed, bounding_box, splinePoints);
          std::sort(splinePoints.begin(), splinePoints.end(), lane_detector::utils::sortPointsY);
          splines.push_back(splinePoints);

          // draw the spline
	         if(config.draw_splines) lane_detector::utils::drawSpline(original, splinePoints, cv::Scalar(255, 0, 0));

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
        /*if(splines.size() == 3) {
          std::cout << "1: " << splines[0].size() << " 2: " << splines[0].size() << " 3: " << splines[1].size() << std::endl;
        }*/

        cv::Point p1;
        cv::Point p2;
        cv::Point p3;
        cv::Point p4;

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

          p1 = longest_spline[0];
          p2 = longest_spline[(longest_spline.size()-1)*0.3333];
          p3 = longest_spline[(longest_spline.size()-1)*0.6666];
          p4 = longest_spline[(longest_spline.size()-1)];

          int middle = 0;
          for(int i = 0; i < longest_spline.size(); i++) {
            if(longest_spline[i].y >= second_longest_spline[0].y) {
              middle = cvRound((longest_spline[i].x - second_longest_spline[0].x)/2);

              //std::cout << "x: " << p1.x << " y: " << p1.y << std::endl;
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

      //cv::line(original, cv::Point(car_position.x, 0), cv::Point(car_position.x, lanesConf.ipmHeight-1), cv::Scalar(0, 255, 239), 1);
}

void Fitting::initTracking(const std::vector<LaneDetector::Box>& ipmBoxes) {
  for(int i = 0; i < 3; i++) {
    cv::KalmanFilter kf = kalmanFilters[i];
    kf.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1/30,0,
                                                      0,1,0,1/30,
                                                      0,0,0,1);
    kf.statePre.at<float>(0) = ipmBoxes[i].line.startPoint.x;
    kf.statePre.at<float>(1) = ipmBoxes[i].line.endPoint.x;
    kf.statePre.at<float>(2) = 0;
    kf.statePre.at<float>(3) = 0;
    setIdentity(kf.measurementMatrix);
    setIdentity(kf.processNoiseCov, cv::Scalar::all(.05));
    setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, cv::Scalar::all(.1));
  }
}

void Fitting::trackBoxes(std::vector<LaneDetector::Box>& ipmBoxes) {
  std::vector<LaneDetector::Box> boxes = ipmBoxes;
  std::vector<LaneDetector::Line> lines;

  for(int i = 0; i < 3; i++) {
    cv::KalmanFilter kf = kalmanFilters[i];
    cv::Mat prediction = kf.predict();
    float x_start = prediction.at<float>(0);
    float x_end = prediction.at<float>(1);
    float min_diff = 9999;
    int min_idx = 0;
    std::cout << "x_end: " << x_end << std::endl;
    if(boxes.size() > 0 ) {
      for(int j = 0; j < boxes.size(); j++) {
        float current_x_end = boxes[j].line.endPoint.x;
        float current_diff = std::fabs(x_end - current_x_end);
        if(current_diff < min_diff) {
          min_diff = current_diff;
          min_idx = j;
        }
      }
      std::cout << "mea: " << boxes[min_idx].line.endPoint.x << std::endl;
      cv::Mat_<float> measurement(2,1);
      measurement(0) = boxes[min_idx].line.startPoint.x;
      measurement(1) = boxes[min_idx].line.endPoint.x;
      cv::Mat estimated = kf.correct(measurement);
      CvPoint2D32f estimated_startPoint = {estimated.at<float>(0), 0};
      CvPoint2D32f estimated_endPoint = {estimated.at<float>(1), lanesConf.ipmHeight-1};
      LaneDetector::Line estimated_line = {estimated_startPoint, estimated_endPoint};
      lines.push_back(estimated_line);
      boxes.erase(boxes.begin()+min_idx);
  }

  else { //The line was missing
    CvPoint2D32f predicted_startPoint = {x_start, 0};
    CvPoint2D32f predicted_endPoint = {x_end, lanesConf.ipmHeight-1};
    LaneDetector::Line predicted_line = {predicted_startPoint, predicted_endPoint};
    lines.push_back(predicted_line);
  }

  ipmBoxes.clear();
  mcvGetLinesBoundingBoxesVec(lines, LaneDetector::LINE_VERTICAL, cvSize(lanesConf.ipmWidth-1, lanesConf.ipmHeight-1), ipmBoxes);
    /*kf.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,1,0,
                                                      0,1,0,1,
                                                      0,0,0,1);
    kf.statePre.at<float>(0) = ipmBoxes[i].line.startPoint.x;
    kf.statePre.at<float>(1) = ipmBoxes[i].line.endPoint.x;
    kf.statePre.at<float>(2) = 0;
    kf.statePre.at<float>(3) = 0;
    setIdentity(kf.measurementMatrix);
    setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
    setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, cv::Scalar::all(.1));*/
  }
}
