/*
 * fitting.h
 *
 *      Author:
 *         Nicolas Acero
 */
#ifndef FITTING_H_
#define FITTING_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <lane_detector/DetectorConfig.h>
#include <cv.h>
#include <lane_detector/LaneDetector.hh>
#include <lane_detector/LaneDetectorOpt.h>
#include <lane_detector/utils.h>
#include <lane_detector/InversePerspectiveMapping.hh>
#include <lane_detector/fittingApproach.h>
#include <lane_detector/lane_tracker/Ctracker.h>

class Fitting {
public:
        inline Fitting(){
          //tracker = CTracker(0.2f, 0.1f, 60.0f, 10, 50);
        };
        inline void setConfig(lane_detector::DetectorConfig& config) {
                this->config = config;
                lane_detector::utils::translateConfiguration(config, this->lanesConf);
                tracker.setDt(config.tracking_dt);
                tracker.setAccelNoiseMag(config.tracking_acc_noise_magnitude);
                tracker.setDistThres(config.tracking_dist_threshold);
                tracker.setMaximumAllowedSkippedFrames(config.tracking_num_absent_frames);
                tracker.setMinimumSeenFrames(config.tracking_num_seen_frames);
                tracker.setMaxTraceLength(50);
        };
        void fitting(cv::Mat& original, cv::Mat& preprocessed, std::vector<LaneDetector::Box>& boxes);
private:
        lane_detector::DetectorConfig config;
        LaneDetector::LaneDetectorConf lanesConf;
        CTracker tracker;
        bool kf_initialized = false;
};

#endif /* FITTING_H_ */
