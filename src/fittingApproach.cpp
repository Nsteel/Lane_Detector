#include <lane_detector/fittingApproach.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <lane_detector/utils.h>
#include <lane_detector/fitSpline.h>
#include <mrpt/math/model_search.h>
#include <mrpt/random.h>


void FittingApproach::fitting(cv::Mat& mat, cv::Rect& box, std::vector<cv::Point>& splinePoints)
{

        box.y = 0;
        box.height = mat.rows-1;
        cv::Rect ransac_box = box;
        //Resize the box for ransac, in order to avoid noise.
        ransac_box += cv::Point(0, 25);
        ransac_box -= cv::Size(0, 50);
        cv::Mat roi = mat.clone();
        cv::Mat ransac_window = mat.clone();
        lane_detector::utils::setMat(roi, box);
        lane_detector::utils::setMat(ransac_window, ransac_box);
        box = ransac_box;
        //std::cout << "8" << std::endl;
        std::vector<TPoint2D> points;

        for(int i=0; i < ransac_window.rows; i++)
        {
                float* pixel = ransac_window.ptr<float>(i);
                for(int j=0; j<ransac_window.cols; j++)
                {
                        float value = pixel[j];
                        //ROS_INFO("Val:%f", value);
                        if(value > 0)
                        {
                          TPoint2D p(ransac_window.rows-1-i, j);
                          points.push_back(p);
                        }
                }
          }

        if(points.size() >= 4) {
          //ROS_INFO("Size: %lu", points.size());
          FitSpline fit(points);
          CSplineInterpolator1D best_model;
          vector_size_t best_inliers;
          ModelSearch search;
          const double DIST_THRESHOLD = 1;
          std::vector<double> best_initial_x;
      		std::vector<double> best_initial_y;

      		bool found = search.ransacSingleModel( fit, 4, DIST_THRESHOLD, best_model, best_inliers );
          //ROS_INFO("Found:%i",found);
          if(found) {
            ROS_DEBUG("Inliers count: %lu", best_inliers.size());
            float start_x = 9999;
            float end_x = -1;
            for(int i : best_inliers) {
              TPoint2D p = points[i];
              float x = p.x;
              if(x < start_x) start_x = x;
              if(x > end_x) end_x = x;
            }
            //ROS_INFO("Start:%f, end:%f", start_x, end_x);
            bool is_inlier = false;
            double y;
            //TODO improve for
            for(double x = start_x; x <= end_x; x++) {
              best_model.query(x, y, is_inlier);
              //ROS_INFO("Inlier:%i, x:%f", is_inlier, x);
              float aux_x = y;
              float aux_y = mat.rows-1-x;
              if(is_inlier) {
                //cv::circle(ransac_window, cv::Point(cvRound(aux_x), cvRound(aux_y)), 2, cv::Scalar(255,0,0), 2);
                splinePoints.push_back(cv::Point(cvRound(aux_x), cvRound(aux_y)));
              }
          }
        }
      }
}
