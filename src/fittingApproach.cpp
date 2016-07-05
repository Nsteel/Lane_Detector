#include <lane_detector/fittingApproach.h>
#include <iostream>
#include <swri_profiler/profiler.h>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <lane_detector/utils.h>
#include <lane_detector/fitSpline.h>
#include <mrpt/math/model_search.h>
#include <mrpt/random.h>


std::vector<cv::Point> FittingApproach::fitting(cv::Mat& original, cv::Mat& preprocessed)
{

        SWRI_PROFILE("Fitting");

        std::vector<TPoint2D> points;

        for(int i=0; i < preprocessed.rows; i++)
        {
                cv::Vec3f* pixel = preprocessed.ptr<cv::Vec3f>(i);
                for(int j=0; j<preprocessed.cols; j++)
                {
                        float value = pixel[j][0];
                        //ROS_INFO("Val:%f", value);
                        if(value > 0)
                        {
                          TPoint2D p(preprocessed.rows-1-i, j);
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
            //cv::cvtColor(preprocessed, preprocessed, CV_GRAY2BGR);
            ROS_DEBUG("Inliers count: %lu", best_inliers.size());
            float start_x = 9999;
            float end_x = -1;
            for(int i : best_inliers) {
              TPoint2D p = points[i];
              float x = p.x;
              //float y = preprocessed.rows-1 - p.y;
              //cv::circle(preprocessed, cv::Point(x, y), 2, cv::Scalar(255,0,0), 2);
              if(x < start_x) start_x = x;
              if(x > end_x) end_x = x;
            }
            //ROS_INFO("Start:%f, end:%f", start_x, end_x);
            bool is_inlier = false;
            double y;
            for(double x = start_x; x <= end_x; x++) {
              best_model.query(x, y, is_inlier);
              //ROS_INFO("Inlier:%i, x:%f", is_inlier, x);
              float aux_x = y;
              float aux_y = preprocessed.rows-1-x;
              if(is_inlier) cv::circle(preprocessed, cv::Point(cvRound(aux_x), cvRound(aux_y)), 2, cv::Scalar(255,0,0), 2);
          }
        }
      }
        /*Returning the points of interest in a vector;
         * using the following format: {vanishing_point, left_lane, right_lane}
         */
        std::vector<cv::Point> points_interest;
        return points_interest;
}
