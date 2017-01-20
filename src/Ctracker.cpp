#include <lane_detector/lane_tracker/Ctracker.h>

//die folgenden Codeabschnitte sind open source und wurden aus https://github.com/Smorodov/Multitarget-tracker genommen

// ---------------------------------------------------------------------------
//param rects input/output bounding boxes
// ---------------------------------------------------------------------------
void CTracker::Update(
	const std::vector<Point_t>& detections,
  const std::vector<cv::Rect>& rects,
	DistType distType
	)
{
	assert(detections.size() == rects.size());

	// -----------------------------------
	// If there is no tracks yet, then every cv::Point begins its own track.
	// -----------------------------------
	if (tracks.size() == 0)
	{
		// If no tracks yet
		for (size_t i = 0; i < detections.size(); ++i)
		{
			tracks.push_back(std::make_shared<CTrack>(detections[i], rects[i], dt, Accel_noise_mag, NextTrackID++));
		}
	}

	size_t N = tracks.size();
	size_t M = detections.size();
	assignments_t assignment;

	if (!tracks.empty())
	{

		distMatrix_t Cost(N * M);

		switch (distType)
		{
		case CentersDist:
			for (size_t i = 0; i < tracks.size(); i++)
			{
				for (size_t j = 0; j < detections.size(); j++)
				{
					Cost[i + j * N] = tracks[i]->CalcDist(detections[j]);
				}
			}
			break;

		case RectsDist:
			for (size_t i = 0; i < tracks.size(); i++)
			{
				for (size_t j = 0; j < detections.size(); j++)
				{
					Cost[i + j * N] = tracks[i]->CalcDist(rects[j]);
				}
			}
			break;
		}

		// -----------------------------------
		// Solving assignment problem (tracks and predictions of Kalman filter)
		// -----------------------------------
		AssignmentProblemSolver APS;
		APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);

		// -----------------------------------
		// clean assignment from pairs with large distance
		// -----------------------------------
		for (size_t i = 0; i < assignment.size(); i++)
		{
			if (assignment[i] != -1)
			{
				//std::cout << "Kalman cost: " << Cost[i + assignment[i] * N] << std::endl;
				if (Cost[i + assignment[i] * N] > dist_thres)
				{
					assignment[i] = -1;
					tracks[i]->skipped_frames++;
				}
			}
			else
			{
				// If track have no assigned detect, then increment skipped frames counter.
				tracks[i]->skipped_frames++;
			}
		}

		// -----------------------------------
		// If track didn't get detects long time, remove it.
		// -----------------------------------
		for (int i = 0; i < static_cast<int>(tracks.size()); i++)
		{
			if (tracks[i]->skipped_frames > maximum_allowed_skipped_frames)
			{
				tracks.erase(tracks.begin() + i);
				assignment.erase(assignment.begin() + i);
				i--;
			}
		}
	}

	// -----------------------------------
    // Search for unassigned detects and start new tracks for them.
	// -----------------------------------
    for (size_t i = 0; i < detections.size(); ++i)
	{
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
		{
			tracks.push_back(std::make_shared<CTrack>(detections[i], rects[i], dt, Accel_noise_mag, NextTrackID++));
		}
	}

	// Update Kalman Filters state

    for (size_t i = 0; i<assignment.size(); i++)
	{
		// If track updated less than one time, than filter state is not correct.

		if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
		{
			tracks[i]->skipped_frames = 0;
      tracks[i]->seen_frames++;
			tracks[i]->Update(detections[assignment[i]], rects[assignment[i]], true, max_trace_length);
		}
		else				     // if not continue using predictions
		{
			tracks[i]->Update(Point_t(), cv::Rect(), false, max_trace_length);
		}
	}

}

std::vector<cv::Rect> CTracker::getLastRects() {
  std::vector<cv::Rect> output_rects;

  for (int i = 0; i < tracks.size(); i++)
  {
    if(tracks[i]->seen_frames >= minimum_seen_frames) {
      output_rects.push_back(tracks[i]->GetLastRect());
    }
  }

  return output_rects;
}
