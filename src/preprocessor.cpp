#include <lane_detector/preprocessor.h>
#include <swri_profiler/profiler.h>

void Preprocessor::preprocess(cv::Mat& img, LaneDetector::IPMInfo& ipmInfo_) {
        SWRI_PROFILE("Preprocess");

        this->ipmInfo = ipmInfo;
        //cv::Mat originalImg = cv::imread("/home/n/Desktop/curve.png");
        CvMat raw_mat = img;
        CvMat* raw_ptr = &raw_mat;
        CvMat* mat_ptr;
        CvMat* fx = cvCreateMat(lanesConf.kernelWidth, 1, FLOAT_MAT_TYPE);
        CvMat* fy = cvCreateMat(lanesConf.kernelHeight, 1, FLOAT_MAT_TYPE);
        LaneDetector::mcvGetGaussianKernel(fy, (lanesConf.kernelHeight-1)/2, lanesConf.lineHeight*ipmInfo.yScale);
        LaneDetector::mcvGet2DerivativeGaussianKernel(fx, (lanesConf.kernelWidth-1)/2, lanesConf.lineWidth*ipmInfo.xScale);
        //LaneDetector::SHOW_MAT(fx, "Kernel_x:");
        //LaneDetector::SHOW_MAT(fy, "Kernel_y:");
        LaneDetector::mcvLoadImage(&raw_ptr, &mat_ptr);
        //subtract mean
        CvScalar mean = cvAvg(mat_ptr);
        cvSubS(mat_ptr, mean, mat_ptr);
        //do the filtering
        cvFilter2D(mat_ptr, mat_ptr, fx); //inImage outImage
        cvFilter2D(mat_ptr, mat_ptr, fy);
        mcvPreprocess(&mat_ptr, &cameraInfo, &ipmInfo, &lanesConf);
        ipmInfo_ = ipmInfo;
        //LaneDetector::mcvScaleMat(mat_ptr, mat_ptr);
        ROS_DEBUG("xScale: %f, yScale: %f, ymaxLim: %f",ipmInfo.xScale, ipmInfo.yScale, ipmInfo.yLimits[1]);
        img = cv::cvarrToMat(mat_ptr, true);
        cvReleaseMat(&mat_ptr);
        cvReleaseMat(&fx);
        cvReleaseMat(&fy);
}
