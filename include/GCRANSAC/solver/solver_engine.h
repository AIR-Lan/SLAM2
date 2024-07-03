

#pragma once

#include <math.h>
#include <chrono>
#include <random>
#include <vector>

#include <opencv2/core/core.hpp>
#include <unsupported/Eigen/Polynomials>
#include <Eigen/Eigen>

#include "GCRANSAC/model.h"

namespace gcransac
{
	namespace estimator
	{
		namespace solver
		{
			// This is a base class for other solvers defined.
			class SolverEngine
			{
			public:
				SolverEngine()
				{
				}

				~SolverEngine()
				{
				}

				// Determines if there is a chance of returning multiple models
				// the function 'estimateModel' is applied.
				static constexpr bool returnMultipleModels()
				{
					return maximumSolutions() > 1;
				}

				static constexpr size_t maximumSolutions()
				{
					return 1;
				}

				// The minimum number of points required for the estimation
				static constexpr size_t sampleSize()
				{
					return 0;
				}

				// Estimate the model parameters from the given point sample
				// using weighted fitting if possible.
				virtual OLGA_INLINE bool estimateModel(
					const cv::Mat &data_,
					const size_t *sample_,
					size_t sample_number_,
					std::vector<Model> &models_,
					const double *weights_ = nullptr) const = 0;
			};
		}
	}
}