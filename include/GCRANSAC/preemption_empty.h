
#pragma once

#include "GCRANSAC/model.h"
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <iostream>

namespace gcransac
{
	namespace preemption
	{
		template <typename _ModelEstimator>
		class EmptyPreemptiveVerfication
		{
		public:
			static constexpr bool providesScore() { return false; }
			static constexpr const char *getName() { return "empty"; }

			bool verifyModel(
				const gcransac::Model &model_,	   // The current model
				const _ModelEstimator &estimator_, // The model estimator
				const double &threshold_,		   // The truncated threshold
				const size_t &iteration_number_,   // The current iteration number
				const Score &best_score_,		   // The current best score
				const cv::Mat &points_,			   // The data points
				const size_t *minimal_sample_,	   // The current minimal sample
				const size_t sample_number_,	   // The number of samples used
				std::vector<size_t> &inliers_,	   // The current inlier set
				Score &score_)					   // The score of the model
			{
				return true;
			}
		};
	}
}