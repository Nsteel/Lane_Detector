#include <lane_detector/preprocessor.h>
#include <swri_profiler/profiler.h>

void Preprocessor::preprocess(cv::Mat& img) {
        SWRI_PROFILE("Preprocess");
        //cv::Mat originalImg = cv::imread("/home/n/Desktop/curve.png");
        CvMat raw_mat = img;
        CvMat* raw_ptr = &raw_mat;
        CvMat* mat_ptr;

        LaneDetector::mcvLoadImage(&raw_ptr, &mat_ptr);
        mcvPreprocess(&mat_ptr, &cameraInfo, &ipmInfo, &lanesConf);

        img = cv::cvarrToMat(mat_ptr, true);
        cvReleaseMat(&mat_ptr);
        //cvReleaseMat(&raw_ptr);
}
