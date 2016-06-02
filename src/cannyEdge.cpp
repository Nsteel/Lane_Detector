#include <lane_detector/cannyEdge.h>
#include <swri_profiler/profiler.h>

//std::vector<cv::Mat> lastImages;

void CannyEdge::preprocess(cv::Mat& img) {
        SWRI_PROFILE("Canny");

        if(img.channels() == 3) {
                cv::cvtColor(img, img, CV_BGR2GRAY);
        }

        img = img(cv::Rect(0, img.rows*config.horizon_percentage/100, img.cols, img.rows*(1-config.horizon_percentage/100)));
        /*cv::Mat filteredImg(cv::Size(img.cols, img.rows*config.horizon_percentage/100), CV_8UC1, cv::Scalar(0));
           if(lastImages.size() == 8) {
           lastImages.erase(lastImages.begin());
           lastImages.push_back(img);
           for(int i = 0; i < filteredImg.rows; i++) {
            std::vector<uchar*> pixels;
            uchar* pixelToFilter = filteredImg.ptr<uchar>(i);
            for(int k = 0; k < lastImages.size(); k++) {
              pixels.push_back(lastImages[k].ptr<uchar>(i));
            }
            for(int j = 0; j < filteredImg.cols; j++) {
              int max = -1;
              for(int f = 0; f < lastImages.size(); f++) {
                if(pixels[f][j] > max) max = pixels[f][j];
              }
              pixelToFilter[j] = max;
            }
           }
           img = filteredImg;
           }
           else {
           lastImages.push_back(img);
           }*/
        cv::Mat r;
        double otsu_thresh_val = cv::threshold(img, r, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        cv::Canny(img,img, otsu_thresh_val*0.5, otsu_thresh_val);
}
