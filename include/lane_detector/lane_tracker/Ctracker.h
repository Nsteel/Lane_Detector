#pragma once
#include <lane_detector/lane_tracker/Kalman.h>
#include <lane_detector/lane_tracker/HungarianAlg.h>
#include <lane_detector/lane_tracker/defines.h>
#include <lane_detector/utils.h>
#include <iostream>
#include <vector>
#include <memory>
#include <array>

// --------------------------------------------------------------------------
class CTrack
{
public:
	CTrack(const Point_t& p, const cv::Rect& rect, track_t dt, track_t Accel_noise_mag, size_t trackID)
		:
		track_id(trackID),
		skipped_frames(0),
		seen_frames(0),
		prediction(p),
		lastRect(rect),
		KF(p, dt, Accel_noise_mag)
	{
	}

	track_t CalcDist(const Point_t& p)
	{
		Point_t diff = prediction - p;
		return sqrtf(diff.x * diff.x + diff.y * diff.y);
	}

	track_t CalcDist(const cv::Rect& r)
	{
		cv::Rect rect = GetLastRect();

		std::array<track_t, 4> diff;
		diff[0] = prediction.x - lastRect.width / 2 - r.x;
		diff[1] = prediction.y - lastRect.height / 2 - r.y;
		diff[2] = static_cast<track_t>(lastRect.width - r.width);
		diff[3] = static_cast<track_t>(lastRect.height - r.height);

		track_t dist = 0;
		for (size_t i = 0; i < diff.size(); ++i)
		{
			dist += diff[i] * diff[i];
		}
		return sqrtf(dist);
	}

	void Update(const Point_t& p, const cv::Rect& rect, bool dataCorrect, size_t max_trace_length)
	{
		KF.GetPrediction();
		prediction = KF.Update(p, dataCorrect);

		if (dataCorrect)
		{
			lastRect = rect;
		}

		if (trace.size() > max_trace_length)
		{
			trace.erase(trace.begin(), trace.end() - max_trace_length);
		}

		trace.push_back(prediction);
	}

	std::vector<Point_t> trace;
	size_t track_id;
	size_t skipped_frames;
	size_t seen_frames;

	cv::Rect GetLastRect()
	{
		return cv::Rect(
			static_cast<int>(prediction.x - lastRect.width / 2),
			static_cast<int>(prediction.y - lastRect.height / 2),
			lastRect.width,
			lastRect.height);
	}

private:
	Point_t prediction;
	cv::Rect lastRect;
	TKalmanFilter KF;
};

// --------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
class CTracker
{
public:
	CTracker(){ NextTrackID = 0; };

	enum DistType
	{
		CentersDist = 0,
		RectsDist = 1
	};

	std::vector<std::shared_ptr<CTrack>> tracks;
	void Update(const std::vector<Point_t>& detections, const std::vector<cv::Rect>& rects, DistType distType);
	std::vector<cv::Rect> getLastRects();
	inline void setDt(track_t dt) { this->dt = dt; };
	inline void setAccelNoiseMag(track_t Accel_noise_mag) { this-> Accel_noise_mag = Accel_noise_mag; };
	inline void setDistThres(track_t dist_thres) { this->dist_thres = dist_thres; };
	inline void setMaximumAllowedSkippedFrames(size_t maximum_allowed_skipped_frames) { this->maximum_allowed_skipped_frames = maximum_allowed_skipped_frames; };
	inline void setMaxTraceLength(size_t max_trace_length) { this->max_trace_length = max_trace_length; };
	inline void setNextTrackID(size_t NextTrackID) { this->NextTrackID = NextTrackID; };
	inline void setMinimumSeenFrames(size_t minimum_seen_frames) { this->minimum_seen_frames = minimum_seen_frames; };

private:
	// Шаг времени опроса фильтра
	track_t dt;

	track_t Accel_noise_mag;

	// Порог расстояния. Если точки находятся дуг от друга на расстоянии,
	// превышающем этот порог, то эта пара не рассматривается в задаче о назначениях.
	track_t dist_thres;
	// Максимальное количество кадров которое трек сохраняется не получая данных о измерений.
    size_t maximum_allowed_skipped_frames;
	// Максимальная длина следа
    size_t max_trace_length;

	size_t NextTrackID;

	size_t minimum_seen_frames;
};
