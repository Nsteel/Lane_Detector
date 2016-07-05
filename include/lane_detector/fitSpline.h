/*
 * fitSpline.h
 *
 *      Author:
 *         Nicolas Acero
 */

 #ifndef FITSPLINE_H_
 #define FITSPLINE_H_

#include <mrpt/math/CSplineInterpolator1D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/math/ransac.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;

struct FitSpline {

  typedef CSplineInterpolator1D		Model;
  typedef double			Real;

	const std::vector<TPoint2D>&  allData;

	FitSpline( const std::vector<TPoint2D>& _allData ) : allData(_allData) {}

	size_t getSampleCount( void ) const
	{
		return allData.size();
	}

	bool  fitModel( const vector_size_t& useIndices, CSplineInterpolator1D& model ) const
	{
		ASSERT_(useIndices.size()==4);
		TPoint2D  p1 (allData[useIndices[0]]);
		TPoint2D  p2 (allData[useIndices[1]]);
		TPoint2D  p3 (allData[useIndices[2]]);
    TPoint2D  p4 (allData[useIndices[3]]);
		std::vector<double> initial_x{p1.x, p2.x, p3.x, p4.x};
		std::vector<double> initial_y{p1.y, p2.y, p3.y, p4.y};

		try
		{
			model = CSplineInterpolator1D(initial_x, initial_y);
		}
		catch(std::exception &)
		{
			return false;
		}

		return true;
	}

	double testSample( size_t index, const CSplineInterpolator1D& model ) const
	{
		bool is_inlier = false;
		TPoint2D p = allData[index];
		double y = 0.0;
		model.query(p.x, y, is_inlier);
		double diff_y = std::fabs(y - p.y);
    //if(isOutlier) std::cout << diff_y << " " << p.x << " " << y << std::endl;
		return is_inlier? diff_y : 999999;
	}
};

#endif /* FITSPLINE_H_ */
