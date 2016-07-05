#include <lane_detector/cannyEdge.h>
#include <swri_profiler/profiler.h>

//std::vector<cv::Mat> lastImages;

void CannyEdge::preprocess(cv::Mat& img) {
        SWRI_PROFILE("Canny");

        //img = img(cv::Rect(0, img.rows*config.horizon_percentage/100, img.cols*0.7, img.rows*(1-config.horizon_percentage/100)));
        cv::normalize(img, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::blur(img , img, cv::Size(2,2));
        //cv::dilate(img, img, cv::Mat());
        cv::threshold(img, img, 170, 255, CV_THRESH_BINARY);

        cv::Canny(img,img, 170, 300);
        //cv::dilate(img, img, cv::Mat());

        std::vector<std::vector<cv::Point> > detectedEdges;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(img, detectedEdges, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        for(int i = 0; i >= 0; i = hierarchy[i][0]) {
                        cv::drawContours(img, detectedEdges, i, cv::Scalar(255), CV_FILLED, 8, hierarchy);
        }
}
