
#pragma once

#include <vector>
#include "GCRANSAC/pearl/GCoptimization.h"

namespace gcransac
{
	namespace estimator
	{
		// Templated class for estimating a model for RANSAC. This class is purely a
		// virtual class and should be implemented for the specific task that RANSAC is
		// being used for. Two methods must be implemented: estimateModel and residual. All
		// other methods are optional, but will likely enhance the quality of the RANSAC
		// output.
		template <typename DatumType, typename ModelType>
		class Estimator
		{
		public:
			typedef DatumType Datum;
			typedef ModelType Model;

			Estimator() {}
			virtual ~Estimator() {}

			// Get the minimum number of samples needed to generate a model.
			OLGA_INLINE virtual size_t inlierLimit() const = 0;

			// Given a set of data points, estimate the model. Users should implement this
			// function appropriately for the task being solved. Returns true for
			// successful model estimation (and outputs model), false for failed
			// estimation. Typically, this is a minimal set, but it is not required to be.
			OLGA_INLINE virtual bool estimateModel(const Datum &data,
												   const size_t *sample,
												   std::vector<Model> *model) const = 0;

			// Estimate a model from a non-minimal sampling of the data. E.g. for a line,
			// use SVD on a set of points instead of constructing a line from two points.
			// By default, this simply implements the minimal case.
			// In case of weighted least-squares, the weights can be fed into the
			// function.
			OLGA_INLINE virtual bool estimateModelNonminimal(const Datum &data,
															 const size_t *sample,
															 const size_t &sample_number,
															 std::vector<Model> *model,
															 const double *weights_ = nullptr) const = 0;

			// Given a model and a data point, calculate the error. Users should implement
			// this function appropriately for the task being solved.
			OLGA_INLINE virtual double residual(const Datum &data, const Model &model) const = 0;
			OLGA_INLINE virtual double squaredResidual(const Datum &data, const Model &model) const = 0;

			// A function to decide if the selected sample is degenerate or not
			// before calculating the model parameters
			OLGA_INLINE virtual bool isValidSample(
				const cv::Mat &data,		// All data points
				const size_t *sample) const // The indices of the selected points
			{
				return true;
			}

			// Enable a quick check to see if the model is valid. This can be a geometric
			// check or some other verification of the model structure.
			OLGA_INLINE virtual bool isValidModel(const Model &model) const { return true; }

			// Enable a quick check to see if the model is valid. This can be a geometric
			// check or some other verification of the model structure.
			OLGA_INLINE virtual bool isValidModel(Model &model,
												  const Datum &data,
												  const std::vector<size_t> &inliers,
												  const size_t *minimal_sample_,
												  const double threshold_,
												  bool &model_updated_) const
			{
				return true;
			}
		};
	}
} // namespace gcransac
